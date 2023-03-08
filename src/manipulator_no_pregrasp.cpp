#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "ROS_print_in_color.h"

#include <robotiq_85_msgs/GripperCmd.h>
#include "project2023/Plan.h"
#include "project2023/Execute.h"
#include "project2023/Ready.h"
#include "design/CreateShelf.h"
#include "ros/ros.h"

#include <memory>

// TODO: place execute에 해당하는 joint value 추가


ros::NodeHandle* nh_ptr = nullptr;

using namespace std;

// 함수 선언문
moveit_msgs::CollisionObject createBoxObject(std::string name, std::vector<double> position, 
                                        tf2::Quaternion quaternion, 
                                        std::vector<double> dimensions, std::string world_frame);

moveit_msgs::CollisionObject addObject(std::string name, std::vector<float> position, 
                                            float obj_radius, float obj_height, std::string world_frame);                                   

class Manipulator{
private:
    moveit::planning_interface::PlanningSceneInterface current_scene;
    std::string reference_frame = "robot_base_link";
    std::string camera_frame = "hand_camera_link";

    std::string obj_name = "cola_can";

    float obj_radius = 0.052/4;     // 콜라캔 직경 = 5.2    
    float obj_H = 0.09;            // 콜라캔 높이 = 11.6    => 0.09

	int execute_state_button = 0;	// 초기값 0 근데 이렇게 하면 안좋은게 place를 무조건 plan뒤에만 할 수 있다. 이건 수정해야 할듯


    // int a;
public:
    moveit::planning_interface::MoveGroupInterface group{"manipulator"};	//main 함수에서 사용할 수 있도록 ptivate => public 으로 이동
	std::string tool = "grasp_link";


    bool ready(){

        // ---------- fake wall 변수 ---------- //
		// manipulator의 기준 좌표를 기준으로 생성됨
        float left_fake_x_coord = 0.5;
        float left_fake_y_coord = 0.55;
        float left_fake_z_coord = 0.6;

        float right_fake_x_coord = 0.5;
        float right_fake_y_coord = -0.50;
        float right_fake_z_coord = 1.2;

        float wall_W = (80.0 - 3.6) * 0.01;        // 전체 width - side plate의 width = back plate의 width
        float wall_D = 2.2 * 0.01;                
        float wall_H = (181.3 - 2.4 - 1.8) * 0.01;
        

        //------ fake wall 생성 ------//
        std::vector<moveit_msgs::CollisionObject> fake_wall;	
        fake_wall.resize(2);		

        fake_wall[0] = createBoxObject("left fake wall",{left_fake_x_coord, left_fake_y_coord, left_fake_z_coord}
                                    ,{0, 0, 0, 1}
                                    ,{wall_W, wall_D, wall_H}
                                    ,reference_frame);

        fake_wall[1] = createBoxObject("right fake wall",{right_fake_x_coord, right_fake_y_coord, right_fake_z_coord}
                                    ,{0, 0, 0, 1}
                                    ,{wall_W, wall_D, wall_H}
                                    ,reference_frame);   

        current_scene.addCollisionObjects(fake_wall);




        //----- Reasy to grasp (Joint value version)-----//
        ROS_GREEN_STREAM("Ready to grasp pose start");
        vector<string> joint_names = {"ur5e_elbow_joint", "ur5e_shoulder_lift_joint", "ur5e_shoulder_pan_joint", "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"};
        vector<double> joint_states = {-2.708991527557373, -1.596262594262594, 2.8748362064361572, 1.1648077207752685, 0.26759424805641174, -1.5719173590289515};
        
        // memo
        // r2g_pose.position.x = r2g_x;
        // r2g_pose.position.y = r2g_y;
        // r2g_pose.position.z = r2g_z;

            // // 90 0 90
        // r2g_pose.orientation.x = 0.5;
        // r2g_pose.orientation.y = -0.5;
        // r2g_pose.orientation.z = 0.5;
        // r2g_pose.orientation.w = 0.5;

        if(joint_names.size() == joint_states.size())
        {
            for(size_t j=0; j<joint_names.size(); j++)
            {
                group.setJointValueTarget (joint_names[j],joint_states[j]);    // setJointValueTarget(const std::string &joint_name, double value)
            }
        }
        
        else
        {
            std::cout << "Wrong input for Joint Targets: name and value count mismatched" << std::endl;
        }

        group.move();
        ROS_GREEN_STREAM("Ready to grasp pose done");
        group.clearPoseTargets();

        // ----- remove fake wall ----- // 
        std::vector<std::string> remove_right_wall;
        remove_right_wall.push_back("right fake wall");
        current_scene.removeCollisionObjects(remove_right_wall);


		ros::ServiceClient client = nh_ptr->serviceClient<design::CreateShelf>("create_shelf");
		design::CreateShelf srv;

		if (client.call(srv))
		{
			ROS_INFO("Send to create_shelf");
		}
		else
		{
			ROS_ERROR("Failed to call service create_shelf");
			return false;
		}
        // this->a = 1;
        return true; 
    };

    bool pick_plan(geometry_msgs::Pose objectPose){
        ROS_GREEN_STREAM("Pick_plan start");
        ROS_GREEN_STREAM(objectPose);

        //------ pick and place 할 object 생성 ------//			매개변수: 이름, 위치(xyz), 반지름값, world_frame
        std::vector<moveit_msgs::CollisionObject> target_object;	// ex) std::vector<moveit_msgs::CollisionObject> Hi : Hi라는 (moveit_msgs::CollisionObject) vector를 만듦.
        target_object.resize(1);									// ex) Hi 라는 vector의 크기를 지정.
        target_object[0] = addObject(obj_name, {objectPose.position.x, objectPose.position.y, objectPose.position.z}
                                    ,obj_radius, obj_H
                                    ,camera_frame);

        current_scene.addCollisionObjects(target_object);

        // 가로 이동 후 앞으로 가는 포즈 planning, planning 넘겨주는것도 해봐야함

        // ----- Reasy to grasp ----- //     앞으로 가는 포즈
        moveit::planning_interface::MoveGroupInterface::Plan pick_plan;

		// --- object의 좌표를 가져오는 코드인데 memo --- //
        // std::string object_name = "cola_can";
        // std::map<std::string, geometry_msgs::Pose> object_poses = current_scene.getObjectPoses({object_name});
        // if (object_poses.size() == 0) {
        //     ROS_ERROR("Object %s not found in the planning scene", object_name.c_str());
        //     return 1;
        // }
        // geometry_msgs::Pose object_pose = object_poses[object_name];
        // ROS_INFO("Object pose: x=%f, y=%f, z=%f", object_pose.position.x, object_pose.position.y, object_pose.position.z);

        geometry_msgs::Pose grasp_pose;

        // 90 0 90
        grasp_pose.position.x = objectPose.position.x;
        grasp_pose.position.y = objectPose.position.y;
        grasp_pose.position.z = objectPose.position.z;
        grasp_pose.orientation.x = objectPose.orientation.x;
        grasp_pose.orientation.y = objectPose.orientation.y;
        grasp_pose.orientation.z = objectPose.orientation.z;
        grasp_pose.orientation.w = objectPose.orientation.w;

        group.setPoseTarget(grasp_pose);
        moveit::planning_interface::MoveItErrorCode success = group.plan(pick_plan);     //Fail: ABORTED: No motion plan found. No execution attempted.
        ROS_WARN_STREAM(success);
        if(!success)
        {
            ROS_WARN_STREAM("Pick planning didn't Succeed\n");
            return false;
        }


        return true; 
    };

    bool place_plan(){

        ROS_GREEN_STREAM("Place_planning start");

        // //----- Reasy to release (Joint value version)-----//
        // ROS_GREEN_STREAM("Ready to grasp pose start");
        // vector<string> joint_names = {"ur5e_elbow_joint", "ur5e_shoulder_lift_joint", "ur5e_shoulder_pan_joint", "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"};
        // vector<double> joint_states = {2.236734215413229, -1.3320907366326828, -0.0004957357989709976, -0.849198119049408, 1.570792555809021, 1.5707550048828125}; // place pose

        // if(joint_names.size() == joint_states.size())
        // {
        //     for(size_t j=0; j<joint_names.size(); j++)
        //     {
        //         group.setJointValueTarget (joint_names[j],joint_states[j]);    // setJointValueTarget(const std::string &joint_name, double value)
        //     }
        // }
        
        // else
        // {
        //     std::cout << "Wrong input for Joint Targets: name and value count mismatched" << std::endl;
        // }

        // // ----- Reasy to release ----- //     앞으로 가는 포즈
        // moveit::planning_interface::MoveGroupInterface::Plan place_plan;
        // geometry_msgs::Pose grasp_pose;


        // // rpy = 90 0 90
        // grasp_pose.position.x = place_x;
        // grasp_pose.position.y = place_y;
        // grasp_pose.position.z = place_z;
        // grasp_pose.orientation.x = 0.5;
        // grasp_pose.orientation.y = -0.5;
        // grasp_pose.orientation.z = 0.5;
        // grasp_pose.orientation.w = 0.5;

        // group.setPoseTarget(grasp_pose);
        // moveit::planning_interface::MoveItErrorCode success = group.plan(place_plan);     //Fail: ABORTED: No motion plan found. No execution attempted.
        // ROS_WARN_STREAM(success);
        // if(!success)
        // {
        //     ROS_WARN_STREAM("Place planning didn't Succeed\n");
        //     return false;
        // }
        return true; 
    };

    bool execute(){
        ros::Publisher gripper_pub = nh_ptr->advertise<robotiq_85_msgs::GripperCmd>("gripper/cmd", 100);   //message type error??????
        group.move();

        // ----- grasp start ----- //
        robotiq_85_msgs::GripperCmd gripper_cmd;
		if (execute_state_button == 0)			// pick execute
		{
            ROS_INFO_STREAM("gripper1");
            gripper_cmd.position = 0.0;		// position range = 0.0 ~ 0.085
            gripper_cmd.speed = 0.05;			// speed range = 0.013 ~ 0.1
            gripper_cmd.force = 0.5;			// force range = 5.0 ~ 220.0
            ROS_GREEN_STREAM(gripper_cmd);
            ros::Rate loop_rate(10);
            while(ros::ok()){
                auto connections = gripper_pub.getNumSubscribers();
                ROS_INFO("Connections: %d", connections);
                if(connections > 0){
                    gripper_pub.publish(gripper_cmd);
                    break;        
                }
                loop_rate.sleep();
            }
            // ros::Duration(1.5).sleep();  // Sleep for one second
		}
		// else if (execute_state_button == 1)		// place execute
		// {
		// 	gripper_cmd.position = 1.0;
		// }



        //----- Reasy to grasp (Joint value version)-----//
        
        vector<string> joint_names = {"ur5e_elbow_joint", "ur5e_shoulder_lift_joint", "ur5e_shoulder_pan_joint", "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"};

        vector<double> joint_states; 

		if (execute_state_button == 1)			// place pose => forward pose
		{
            ROS_INFO_STREAM("button1");
			joint_states = {2.236734215413229, -1.3320907366326828, -0.0004957357989709976, -0.849198119049408, 1.570792555809021, 1.5707550048828125}; // place pose

            if(joint_names.size() == joint_states.size())
            {
                for(size_t j=0; j<joint_names.size(); j++)
                {
                    group.setJointValueTarget (joint_names[j],joint_states[j]);    // setJointValueTarget(const std::string &joint_name, double value)
                }
            }
            
            else
            {
                std::cout << "Wrong input for Joint Targets: name and value count mismatched" << std::endl;
            }
            ROS_INFO_STREAM("Execute start");
            group.move();
            group.clearPoseTargets();

            ROS_INFO_STREAM("Execute done");
            ros::Duration(0.5).sleep();

            gripper_cmd.position = 1.0;
            gripper_cmd.speed = 0.05;			// speed range = 0.013 ~ 0.1
            gripper_cmd.force = 0.5;			// force range = 5.0 ~ 220.0

            ROS_GREEN_STREAM(gripper_cmd);
            ros::Rate loop_rate(10);
            while(ros::ok()){
                auto connections = gripper_pub.getNumSubscribers();
                ROS_INFO("Connections: %d", connections);
                if(connections > 0){
                    gripper_pub.publish(gripper_cmd);
                    break;        
                }
                loop_rate.sleep();
            }
            ROS_GREEN_STREAM("Publish completed.");
            ros::Duration(1.5).sleep();  // Sleep for one second
        }

        else if (execute_state_button == 0)
        {
            ROS_INFO_STREAM("button0");
            //----- Reasy to grasp (Joint value version)-----//
            ROS_GREEN_STREAM("Ready to grasp pose start");
            joint_states = {-2.708991527557373, -1.596262594262594, 2.8748362064361572, 1.1648077207752685, 0.26759424805641174, -1.5719173590289515};
        
            // memo
            // r2g_pose.position.x = r2g_x;
            // r2g_pose.position.y = r2g_y;
            // r2g_pose.position.z = r2g_z;

                // // 90 0 90
            // r2g_pose.orientation.x = 0.5;
            // r2g_pose.orientation.y = -0.5;
            // r2g_pose.orientation.z = 0.5;
            // r2g_pose.orientation.w = 0.5;

            if(joint_names.size() == joint_states.size())
            {
                for(size_t j=0; j<joint_names.size(); j++)
                {
                    group.setJointValueTarget (joint_names[j],joint_states[j]);    // setJointValueTarget(const std::string &joint_name, double value)
                }
            }
            
            else
            {
                std::cout << "Wrong input for Joint Targets: name and value count mismatched" << std::endl;
            }

            group.move();
            ROS_GREEN_STREAM("Ready to grasp pose done");
            group.clearPoseTargets();
            this->execute_state_button = 1;		// 다음에 execute 할때 place로 execute하게.
        }







        //-----  drive pose (Joint value version)-----//
        ROS_GREEN_STREAM("Drive pose start");
        joint_states = {2.5272844473468226, -2.820700784722799, -3.3680592672169496e-05, 0.3486700493046264, 1.5712240934371948, 1.5706626176834106};   // drive pose
        if(joint_names.size() == joint_states.size())
        {
            for(size_t j=0; j<joint_names.size(); j++)
            {
                group.setJointValueTarget (joint_names[j],joint_states[j]);    // setJointValueTarget(const std::string &joint_name, double value)
            }
        }
        
        else
        {
            std::cout << "Wrong input for Joint Targets: name and value count mismatched" << std::endl;
        }
        ROS_INFO_STREAM("Initializing robot pose");
        group.move();
        ROS_INFO_STREAM("Group.move end");
        group.clearPoseTargets();
        return true; 
    };


};

//Manipulator manipulator = Manipulator();
std::shared_ptr<Manipulator> manipulatorPtr;


bool ready_srv(project2023::Ready::Request& req, project2023::Ready::Response& res)
{
    //res.status= manipulator.pick(req.pose);
    res.status = manipulatorPtr->ready();
    return true;
}


bool plan_srv(project2023::Plan::Request& req, project2023::Plan::Response& res)
{
    if (req.task == "pickplan")
        res.status = manipulatorPtr->pick_plan(req.target_pose);
    else if (req.task == "placeplan")
        res.status = manipulatorPtr->place_plan();
    return true;
}

bool execute_srv(project2023::Execute::Request& req, project2023::Execute::Response& res)
{
    res.status = manipulatorPtr->execute();
    return true;
}



//----- create box object function -----//	매개변수: 이름, 위치(xyz), quaternion, 크기, world_frame
moveit_msgs::CollisionObject createBoxObject(std::string name, std::vector<double> position, 
                                        tf2::Quaternion quaternion, 
                                        std::vector<double> dimensions, std::string world_frame)
{
    moveit_msgs::CollisionObject table_box;
    table_box.id = name;
    table_box.header.frame_id = world_frame;  // reference = 기준점

    table_box.primitives.resize(1);
    table_box.primitives[0].type = table_box.primitives[0].BOX;
    table_box.primitives[0].dimensions.resize(3);
    //table_box.primitives[0].dimensions = dimensions;
    table_box.primitives[0].dimensions[0] = dimensions[0];
    table_box.primitives[0].dimensions[1] = dimensions[1];
    table_box.primitives[0].dimensions[2] = dimensions[2];
    /* Define the pose of the table. */
    table_box.primitive_poses.resize(1);
    table_box.primitive_poses[0].position.x = position[0];
    table_box.primitive_poses[0].position.y = position[1];
    table_box.primitive_poses[0].position.z = position[2];
    //xyzw
    table_box.primitive_poses[0].orientation.x = quaternion[0];
    table_box.primitive_poses[0].orientation.y = quaternion[1];
    table_box.primitive_poses[0].orientation.z = quaternion[2];
    table_box.primitive_poses[0].orientation.w = quaternion[3];
    table_box.operation = table_box.ADD;
    return table_box;
}

//----- create target object -----//		매개변수: 이름, 위치(xyz), 반지름값, world_frame
moveit_msgs::CollisionObject addObject(std::string name, std::vector<float> position, 
                                        float obj_radius, float obj_height, std::string world_frame)				// std::double는 오류가남. why?
{
    moveit_msgs::CollisionObject out_obj;
    out_obj.id = name;
    out_obj.header.frame_id = world_frame;  // reference = 기준점

    out_obj.primitives.resize(1);
    out_obj.primitives[0].type = out_obj.primitives[0].CYLINDER;
    out_obj.primitives[0].dimensions.resize(3);
    //out_obj.primitives[0].dimensions = dimensions;
    out_obj.primitives[0].dimensions[0] = obj_height;	//CYLINDER_HEIGHT
    out_obj.primitives[0].dimensions[1] = obj_radius;	//CYLINDER_RADIUS를 함수 매개변수 radius를 통해 값을 받는다.

    /* Define the pose of the table. */
    out_obj.primitive_poses.resize(1);
    out_obj.primitive_poses[0].position.x = position[0];
    out_obj.primitive_poses[0].position.y = position[1];
    out_obj.primitive_poses[0].position.z = position[2];    //I did +1 and everyone except buffer went up by 1 // because they use this function
    //xyzw	xyz는 0값이 들어갈꺼라 생략됨
    out_obj.primitive_poses[0].orientation.x = 0;
    out_obj.primitive_poses[0].orientation.y = -0.7071068;
    out_obj.primitive_poses[0].orientation.z = 0;
    out_obj.primitive_poses[0].orientation.w = 0.7071068;

    out_obj.operation = out_obj.ADD;
    return out_obj;
}


int main(int argc, char **argv)
{
    std::cout << "here1" <<std::endl;
    ros::init(argc, argv, "shelf_model");
    ros::NodeHandle nh;
    std::cout << "here2" <<std::endl;

    nh_ptr = &nh;
    manipulatorPtr = std::make_shared<Manipulator>();

    manipulatorPtr->group.setPlanningTime(0.5);
    manipulatorPtr->group.setNumPlanningAttempts(10.0);
    manipulatorPtr->group.setMaxVelocityScalingFactor(0.1);
    manipulatorPtr->group.setMaxAccelerationScalingFactor(0.1);
    std::cout << "here3" <<std::endl;

    ros::AsyncSpinner spin(0);
    spin.start();

    std::cout << "here4" <<std::endl;

    ros::ServiceServer ready_service = nh.advertiseService("ready", ready_srv);
    ros::ServiceServer plan_service = nh.advertiseService("plan", plan_srv);		// ("서비스 이름", call back 함수)
    ros::ServiceServer execute_service = nh.advertiseService("execute",execute_srv);
    // ros::ServiceServer place_service = nh.advertiseService("plan", plan_srv);
    // ros::ServiceServer service3 = nh.advertiseService("execute", execute_srv);
    ros::waitForShutdown();


    // ros::spin();	// 왜 ros spin을 쓰면 안되는지 설명

    return 0;
}
