//#include <iostream>
//#include <kdl/frames.hpp>
//
////using namespace KDL;
//using namespace std;
//int main(int argc, char** argv)
//{
////    Vector v = Vector(1,2,3);
//    KDL::Vector v1(1,2,3); //隐式创建对象
//    for(int i=0; i<3; ++i){
//        cout<<v1[i]<<";";
//    }
//
//    cout<<"\n";
//    KDL::Vector v0 = KDL::Vector::Zero(); //显式创建对象
//    KDL::Vector vec; //隐式创建对象，调用默认构造函数，成员数据被初始化为0
//    cout<<vec.x()<<";"<<vec.y()<<";"<<vec.z()<<"\n";
////    std::cout<<v.data[0]<<std::endl; //获取数组的第一个元素
//    for(int i=0; i<3; ++i){
//        cout<<v0(i)<<" "; //循环输出数组的元素
//    }
//    cout<<"\n";
//    cout<<v0.x()<<" "; //获取数组的第一个元素
//    cout<<v0[0]; //获取数组的第一个元素
//    return 0;
//}

#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

using namespace KDL;

    Chain Puma560(){
        Chain puma560;
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        return puma560;
    }


int main() {
    Chain puma560;

    puma560 = Puma560();
    ChainFkSolverPos_recursive fwdkin(puma560);
    int n = puma560.getNrOfJoints();
    JntArray q(n);
    Frame pos_goal;

    std::cout << n << std::endl;
//    q.data.setRandom();
    q.data <<0,30,0,0,0,0;
    std::cout<<"q.data  =  "<<q.data<<"\n";

    q.data *= M_PI;
    fwdkin.JntToCart(q, pos_goal);
    std::cout<<"pos_goal.data  =  "<< pos_goal.p(0) ;
    std::cout<<" "<<pos_goal.p(1) <<"\n";
    std::cout<<" "<<pos_goal.p(2) <<"\n";
    std::cout<<"pos_goal.data  =  "<< pos_goal.M.data ;

    Eigen::Matrix<double, 6, 1> L;
    L(0) = 1;
    L(1) = 1;
    L(2) = 1;
    L(3) = 0.01;
    L(4) = 0.01;
    L(5) = 0.01;
    ChainIkSolverPos_LMA solver(puma560, L);
    JntArray q_init(n);
    JntArray q_sol(n);
    q_init.data.setRandom();
    q_init.data *= M_PI;
    int retval;
    retval = solver.CartToJnt(q_init, pos_goal, q_sol);


}