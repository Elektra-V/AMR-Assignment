# AMR_WS_2024_submission
this repo is for the project and code version control for the project to be done is AMR course.



Project Objectives

===============

The objective of this project is that you deploy some of the functionalities that were discussed during the course on a real robot platform. In particular, we want to have functionalities for path and motion planning, localisation, and environment exploration on the robot.

We will particularly use the Robile platform during the project; you are already familiar with this robot from the simulation you have been using throughout the semester as well as from the few practical lab sessions that we have had.

Task Description

==============

The project consists of three parts that are building on each other: (i) path and motion planning, (ii) localisation, and (iii) environment exploration.

1. Path and Motion Planning

You have already implemented a *potential field planner* in one of your assignments. In this first part of the project, you need to port your implementation to the real robot and ensure that it is working as well as it was in the simulated environment so that you can navigate towards global goals while avoiding obstacles.

2. Localisation

In one of the course lectures, we discussed Monte Carlo localisation as a practical solution to the robot localisation problem in an existing map. In this second part of the project, your objective is to implement your very own particle filter that you then integrate on the Robile. You should implement the simple version of the filter that we discussed in the lecture; however, if you have time and interest, you are free to additionally explore extensions / improvements to the algorithm, for example in the form of the adaptive Monte Carlo approach that we mentioned in the lecture.

3. Environment Exploration

The final objective of the project is to incorporate an environment exploration functionality to the robot. This will have to be combined with a SLAM component, namely you will need your exploration component to select poses to explore and a SLAM component that will take care of actually creating a map. The exploration algorithm should ideally select poses at the map fringe (i.e. poses that are at the boundary between the explored and unexplored region), but you are free to explore different pose selection strategies in your implementation.

Practical Notes and Assumptions

===========================

* For the first two tasks in this project, we need an environment map to be given. For this purpose, you should use an already existing SLAM approach in ROS (such as the `slam_toolbox` that you also used to map simulated environments) to create a map of the environment where you conduct your tests.
* You should also use an existing SLAM approach for the last part of the project, such that this will need to run in parallel with the exploration component. The selection of poses should thus be done with respect to the most up-to-date map provided by the SLAM algorithm.

Submission Guidelines

===================

Your submission should be a single text file containing:
A URL of a repository with all the code that you have developed during the project. Make sure that the repository contains a README file to explain the contents of the repository and provide usage guidelines.
URLs to videos demonstrating your developed functionalities on the real Robile platform (you can upload these videos anywhere, for example to Google Drive or YouTube). In the videos, make sure that you explicitly show that you are executing your components!
The file should be uploaded to LEA before the submission deadline. The grading of the project will be done on the basis of this submission.

Demonstration

============

After the submission deadline, each group will also need to present their results in a live demonstration. We will agree on a date for the demonstration at a later date. The live demonstration does not count towards the project grade; it is just there so that you get some live demo experience and so that we can discuss any concrete issues that you have faced in your implementations.

Formalities

=========

Deadline: March 15th, 2024, 23:59 CET
Project demonstration: ~March 20th, 2024
