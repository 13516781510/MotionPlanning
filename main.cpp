#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateSpace.h>       // For ob::StateSpace
#include <ompl/base/ScopedState.h>      // For ob::ScopedState
#include <ompl/base/spaces/SE3StateSpace.h> // For ob::SE3StateSpace
#include <ompl/base/PlannerStatus.h>    // For ob::PlannerStatus
#include <iostream>                     // For std::cout
#include <fstream>                      // For std::ofstream
#include <windows.h>
// 定义一个简单的障碍物
struct Obstacle {
	double x_min, x_max, y_min, y_max, z_min, z_max;
};
Obstacle obstacle = {-0.5, 0.5, -0.5, 0.5, -0.5, 0.5};
namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state);

bool isStateValid(const ob::State *state) {
	const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
	const double x = se3state->getX();
	const double y = se3state->getY();
	const double z = se3state->getZ();

	// 检查状态是否在障碍物外部
	if (x >= obstacle.x_min && x <= obstacle.x_max &&
	    y >= obstacle.y_min && y <= obstacle.y_max &&
	    z >= obstacle.z_min && z <= obstacle.z_max) {
		return false; // 状态在障碍物内部，无效
	}

	return true; // 状态有效
}

void savePathToFile(const og::PathGeometric &path, const std::string &filename) {
	std::ofstream ofs(filename);
	for (std::size_t i = 0; i < path.getStateCount(); ++i) {
		const auto *se3state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
		ofs << se3state->getX() << " " << se3state->getY() << " " << se3state->getZ() << std::endl;
	}
	ofs.close();
}

int main() {

	SetConsoleCP(CP_UTF8);
	auto space(std::make_shared<ob::SE3StateSpace>());
	ob::RealVectorBounds bounds(3);
	bounds.setLow(-1);
	bounds.setHigh(1);
	space->setBounds(bounds);
	og::SimpleSetup ss(space);
	ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });


	ob::ScopedState<ob::SE3StateSpace> start(space);
	ob::ScopedState<ob::SE3StateSpace> goal(space);
	// 手动设置起点和终点
	start->setX(0.3);
	start->setY(1);
	start->setZ(-0.3);
	start->rotation().setIdentity(); // 设置为单位四元数

	goal->setX(-0.8);
	goal->setY(-0.9);
	goal->setZ(-0.7);
	goal->rotation().setIdentity(); // 设置为单位四元数

	// 检查起点和终点的有效性
	if (!isStateValid(start.get())) {
		std::cerr << "Start state is invalid!" << std::endl;
		return -1;
	}

	if (!isStateValid(goal.get())) {
		std::cerr << "Goal state is invalid!" << std::endl;
		return -1;
	}

	ss.setStartAndGoalStates(start, goal);
	ob::PlannerStatus solved = ss.solve(10);
	if (solved) {
		std::cout << "Found solution:" << std::endl;
		// print the path to screen
		ss.simplifySolution();
		ss.getSolutionPath().print(std::cout);
		const og::PathGeometric &path = ss.getSolutionPath();
		savePathToFile(path, "D:/forprogram/Python/ompltest/path.txt");
	}
	return 0;
}

