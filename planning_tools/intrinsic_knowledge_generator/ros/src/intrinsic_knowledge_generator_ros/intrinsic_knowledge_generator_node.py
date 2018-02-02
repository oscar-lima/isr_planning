#!/usr/bin/env python
import rospy
import mbot_world_model_ros.gpsr_dict


class generate_knowledge(object):
	def __init__(self):
		# get the save file path from the parameter server
		self.path = rospy.get_param("/min_req_facts_path", "/home/carlos/ros_ws/src/isr_planning/planning_examples/planning_domains/common/monarch_robot/pddl/gpsr/problems")
		# get the dictionary from the gpsr_dict file under mbot_world_model
		self.slots_dict = mbot_world_model_ros.gpsr_dict.slots_dict

	def gen_min_req_facts(self):
		mrf_file = open(self.path+'minimum_required_facts.pddl', 'w')

		mrf_file.write('(define (problem minimum_required_facts)\n')
		mrf_file.write('\t(:domain gpsr) ; General Purpose Service Robot (in a home environment)\n')
		mrf_file.write('\t(:objects\n')

		# insert all the locations into the file
		mrf_file.write('\n\t\t; locations\n\t\t')
		count = 0
		for key, value in self.slots_dict.items():
			if (value == 'l'):
				mrf_file.write(' l--')
				mrf_file.write(key)
				count = count + 1

			if count == 10:
				mrf_file.write(' - location\n')
				mrf_file.write('\t\t')
				count = 0

		if count != 0:
			mrf_file.write(' - location\n')

		# insert all the persons into the file
		mrf_file.write('\n\t\t; people\n\t\t')
		count = 0
		for key, value in self.slots_dict.items():
			if (value == 'p'):
				mrf_file.write(' p--')
				mrf_file.write(key)
				count = count + 1

			if count == 10:
				mrf_file.write(' - person\n')
				mrf_file.write('\t\t')
				count = 0

		if count != 0:
			mrf_file.write(' - person\n')

		# insert all the objects into the file
		mrf_file.write('\n\t\t; objects\n\t\t')
		count = 0
		for key, value in self.slots_dict.items():
			if (value == 'obj'):
				mrf_file.write(' obj--')
				mrf_file.write(key)
				count = count + 1

			if count == 10:
				mrf_file.write(' - object\n')
				mrf_file.write('\t\t')
				count = 0

		if count != 0:
			mrf_file.write(' - object\n')

		mrf_file.write('\n)\n')

		mrf_file.write('\t(:init\n')
		mrf_file.write('\t\t; the robot at start is in the entrance of the house\n')
		mrf_file.write('\t\t(at_r l--start)\n')

		mrf_file.write('\t\t; the robot gripper is empty at the start\n')
		mrf_file.write('\t\t(gripper_empty)\n')
		mrf_file.write('\t)\n')
		mrf_file.write('\t(:goal )\n')
		mrf_file.write(')\n')

		mrf_file.close()

def main():
	# instantiate the generate_knowledge class in the gen_knowl object
	gen_knowl = generate_knowledge()

	# generate the minimum required facts file
	gen_knowl.gen_min_req_facts()
