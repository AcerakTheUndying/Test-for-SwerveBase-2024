#pragma once
/*
* This file is used to select the robot that code is being pushed to.
* Because this filename is added to the .gitignore file in the root of the Repo
* The changes to this file won't be commited to Github when people make commits
* This means if someone else changes the files and pushs to Github and someone 
* else does a Github sync, this file won't be over written on the programmers laptop
* meaning if you set it once it stays set until you change it.
*/
//define ROBOT2024       //2024 Robot
//#define SWERVYB       //2023 Robot
//#define SWERVY        //2022 Robot
#define SPOBOT        //SPO Robot