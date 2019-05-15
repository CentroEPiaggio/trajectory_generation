#include "trajectory_generation/trajectory_generation_server.h"

trajectory_generation_server::trajectory_generation_server() {

};

trajectory_generation_server::~trajectory_generation_server() {

};


bool trajectory_generation_server::trajectory_generation_call(trajectory_generation::TrajectoryGeneration::Request  &req,
         trajectory_generation::TrajectoryGeneration::Response  &res) {


    std::cout << "Start" << std::endl;
    std::vector<geometry_msgs::Point> pos_vec;
	std::vector<double> t_w,t,x_w,y_w,z_w,x,y,z;

    for(auto& value: req.pos_way_points)
    {
        pos_vec.push_back(value);
    }   

    int pos_way_point_num = pos_vec.size();

    double count = 0;
    for (auto& it: pos_vec) {
    	
    	geometry_msgs::Point p = it;
    	x_w.push_back(p.x);
    	y_w.push_back(p.y);
    	z_w.push_back(p.z);
    	t_w.push_back(count*req.time/(pos_way_point_num-1));
        count++;
    }
    
    for(double j=0;j<req.time+req.Ts;j+=req.Ts) {
    	t.push_back(j);
    }

    pchip(t_w,x_w,t,x);
    pchip(t_w,y_w,t,y);
    pchip(t_w,z_w,t,z);


    // std::ofstream myfile ("//home/alessandro/trajectory_pos.txt");
    // if (myfile.is_open())
    // {
    // for(int k=0;k<t.size();k++){    
    //     myfile << x.at(k) << " " << y.at(k) << " " <<z.at(k) << "\n";
    // }
    //     myfile.close();
    // }
    
    std::vector<geometry_msgs::Quaternion> quat_vec;

    for(auto const& value2: req.or_way_points)
    {
        quat_vec.push_back(value2);
    }   

    std::vector<geometry_msgs::Quaternion> quat;
	double step_or = req.Ts*(quat_vec.size()-1)/req.time;

    generate_slerp(quat_vec.at(0),quat_vec.at(1),step_or,quat);

    for (std::vector<geometry_msgs::Quaternion>::iterator it = quat_vec.begin()+1; it != quat_vec.end()-1; ++it) {
        
        std::vector<geometry_msgs::Quaternion> quat_aux;

        generate_slerp(*it,*(it+1),step_or,quat_aux);
        quat.insert(
            quat.end(),
            std::make_move_iterator(quat_aux.begin()+1),
            std::make_move_iterator(quat_aux.end()));//Joint 7
}
     
    // std::ofstream myfile_or ("//home/alessandro/trajectory_or.txt");
    // if (myfile_or.is_open())
    // {
    // for(int k=0;k<t.size();k++){    
    //     myfile_or << quat.at(k).x << " " << quat.at(k).y << " " <<quat.at(k).z << " " <<quat.at(k).w << "\n";
    // }
    //     myfile_or.close();
    // }
    

    



    for(int k=0;k<t.size();k++){
        geometry_msgs::Pose p;
        p.position.x = x.at(k);
        p.position.y = y.at(k);
        p.position.z = z.at(k);
        p.orientation.x = quat.at(k).x;
        p.orientation.y = quat.at(k).y;
        p.orientation.z = quat.at(k).z;
        p.orientation.w = quat.at(k).w;

        
        res.trajectory.push_back(p);
    }


    std::cout << "Finished" << std::endl;
    return true;
};



void trajectory_generation_server::generate_slerp(geometry_msgs::Quaternion q_in,geometry_msgs::Quaternion q_fin,double step, std::vector<geometry_msgs::Quaternion> &q_int) 
{

    tf::Quaternion quat_0(q_in.x,q_in.y,q_in.z,q_in.w);
    tf::Quaternion quat_1(q_fin.x,q_fin.y,q_fin.z,q_fin.w);
    tf::Quaternion inter;

    double num_of_samp = 1/step +1;
    for(double j=0;j<=num_of_samp;j++)
    {
        inter=quat_0.slerp(quat_1,j/num_of_samp);
        inter.normalize();
        geometry_msgs::Quaternion q_msg;
        q_msg.w = inter.w();
        q_msg.x = inter.x();
        q_msg.y = inter.y();
        q_msg.z = inter.z();

        q_int.push_back(q_msg);
    }
};
