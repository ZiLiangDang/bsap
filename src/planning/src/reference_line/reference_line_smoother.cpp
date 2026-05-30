#include "reference_line_smoother.h"

namespace Planning
{
    ReferenceLineSmoother::ReferenceLineSmoother() // 参考线平滑
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_smoother created");

        // 读取配置文件
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();
    }

    void ReferenceLineSmoother::smooth_reference_line(Referline &refer_line)// 平滑参考线
    {
        const int n = refer_line.refer_line.size();
        if (n < 3)
        {
            return; // 如果参考线点数不足3个，则不进行平滑
        }

        //造p矩阵：
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d W1 = 2.0 * w1 * I;
        Eigen::Matrix2d W2 = 2.0 * w2 * I;
        Eigen::Matrix2d W3 = 2.0 * w3 * I;

        Eigen::Matrix2d block1 = W1 + W2 + W3;
        Eigen::Matrix2d block2 = -2.0 * W1 - W2;
        Eigen::Matrix2d block3 = -4.0 * W1 - W2;
        Eigen::Matrix2d block4 = 5.0 * W1 + 2.0 * W2 + W3;
        Eigen::Matrix2d block5 = 6.0 * W1 + 2.0 * W2 + W3;

        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(2 * n, 2 * n);//初始化为零矩阵

        if(n == 3)//单独处理n==3的情况
        {
            // 上三角部分
            //|W1+W2+W3  -2W1-W2         W1|
            //|          4W1+2W2+W3 -2W1-W2|
            //|                    W1+W2+W3|
            
            //只填充上三角部分
            P_tmp.block<2, 2>(0, 0) = block1;
            P_tmp.block<2, 2>(0, 2) = block2;
            P_tmp.block<2, 2>(0, 4) = W1;
            P_tmp.block<2, 2>(2, 2) = 4 * W1 + 2.0 * W2 + W3;
            P_tmp.block<2, 2>(2, 4) = block2;
            P_tmp.block<2, 2>(4, 4) = block1; 
        }
        else
        {
            // | W1 + W2 + W3,     -2W1  - W2,             W1,            0,              0,      ...      0  |
            // |               5W1 + 2W2 + W3,      -4W1 - W2,           W1,              0,      ...      0  |
            // |                               6W1 + 2W2 + W3,    -4W1 - W2,             W1,     ...       0  |
            // |                                                          .               .                .  |
            // |                                                          .               .                .  |
            // |                                                          .               .                .  |
            // |                                             6W1 + 2W2 + W3,       -4W1 - W2,              W1 |
            // |                                                              5W1 + 2W2 + W3,        -2W1 - W2|
            // |                                                                                 W1 + W2 + W3 |                  
            //填充上三角部分
            for(int i = 0; i < n; ++i)
            {
                if(i==0)//第0行
                {
                    P_tmp.block<2,2>(2*i, 2*i) = block1;
                    P_tmp.block<2,2>(2*i, 2*(i+1)) = block2;
                    P_tmp.block<2,2>(2*i, 2*(i+2)) = W1;
                }
                else if(i==1)//第一行
                {
                    P_tmp.block<2,2>(2*i, 2*i) = block4;
                    P_tmp.block<2,2>(2*i, 2*(i+1)) = block3;
                    P_tmp.block<2,2>(2*i, 2*(i+2)) = W1;
                }
                else if(i==n-2)//倒数第二行
                {
                    P_tmp.block<2,2>(2*i, 2*i) = block4;
                    P_tmp.block<2,2>(2*i, 2*(i+1)) = block2;
                }
                else if(i==n-1)//最后一行
                {
                    P_tmp.block<2,2>(2*i, 2*i) = block1;
                }
                else//中间其他行
                {
                    P_tmp.block<2,2>(2*i, 2*i) = block5;
                    P_tmp.block<2,2>(2*i, 2*(i+1)) = block3;
                    P_tmp.block<2,2>(2*i, 2*(i+2)) = W1;
                }            
            }
        }

        P_tmp = P_tmp.selfadjointView<Eigen::Upper>(); // 取上三角部分
        Eigen::SparseMatrix<double> P = P_tmp.sparseView(); // 转换为稀疏矩阵

        Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(2 * n, 2 * n);
        Eigen::SparseMatrix<double> A = A_tmp.sparseView();//转化为稀疏矩阵

        //原始点坐标
        Eigen::VectorXd X(2 * n);
        for (int i = 0; i < n; i++)
        {
            X(2 * i) = refer_line.refer_line[i].pose.pose.position.x;
            X(2 * i + 1) = refer_line.refer_line[i].pose.pose.position.y;
        }

        Eigen::VectorXd Q = -2.0*X;//一次项矩阵
        Eigen::VectorXd buff = Eigen::VectorXd::Constant(2 * n, 0.2);//偏差范围，动态列向量，2*n行，1列，值全为0.2
        buff(0) = buff(1) = buff(2*n-2) = buff(2*n-1)  = 0.0;//首尾点坐标不允许偏移
        Eigen::VectorXd lowerBound = X - buff;//不等式约束下界
        Eigen::VectorXd upperBound = X + buff;//不等式约束上界

        // 创建OSQP求解器
        OsqpEigen::Solver solver;
        // 设置
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        // 初始化求解器
        solver.data()->setNumberOfVariables(2*n);//变量数
        solver.data()->setNumberOfConstraints(2*n);//约束数
        if (!solver.data()->setHessianMatrix(P))
        {
            return;
        }
        if (!solver.data()->setGradient(Q))
        {
            return;
        }
        if (!solver.data()->setLinearConstraintsMatrix(A))
        {
            return;
        }
        if (!solver.data()->setLowerBound(lowerBound))
        {
            return;
        }
        if (!solver.data()->setUpperBound(upperBound))
        {
            return;
        }
        if (!solver.initSolver())
        {
            return;
        }
        // 求解
        Eigen::VectorXd QPSolution;//解向量
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            return;
        }
        QPSolution = solver.getSolution();

        //把结果向量中的数据更新到refer_line中
        for (int i = 0; i < n; i++)
        {
            refer_line.refer_line[i].pose.pose.position.x = QPSolution(2 * i);
            refer_line.refer_line[i].pose.pose.position.y = QPSolution(2 * i + 1);
        }
    }

}// namespace Planning