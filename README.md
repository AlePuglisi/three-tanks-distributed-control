# three-tanks-distributed-control
MATLAB | YALMIP | Centralized,Decentralized,Distributed Control | Linear Matrix Inequality (LMI) | Control Theory

This repository contains the MATLAB code and Report of the final group project of a course on [Networked Control systems](https://www11.ceda.polimi.it/schedaincarico/schedaincarico/controller/scheda_pubblica/SchedaPublic.do?&evn_default=evento&c_classe=810213&__pj0=0&__pj1=7f724b397a9ae97bd6cf089503f5774c).

..WIP..

## Introduction 
In this project, we solve a benchmark problem in control theory, the (linearized) modeling and control of a system of three tanks in series.<br/>
In particular, coherently with the course topics, we implement different control schemes (centralized, decentralized, distributed).<br/>
Simulation and performance analysis are done both in continuous-time and discrete-time domain.<br/>

<image width=390 height=280 src=https://github.com/user-attachments/assets/ed309489-9679-4e45-af91-1218c316ddf7>
<image width=420 height=280 src=https://github.com/user-attachments/assets/4a309df1-49a2-4c43-bc94-6c02ca16e1cc>
  
<br/>
<br/>
The project is implemented in MATLAB, and YALMIP (with sedumi or SDTP3 for low-level optimization) as the optimization toolbox for LMI.<br/>
Controller tuning relies on Linear Matrix Inequality (LMI) theory (simple but efficient). 

<br/>
<br/>

To have further details on the modeling, system decomposition, and control tuning, refer to our [presentation](https://github.com/AlePuglisi/three-tanks-distributed-control/blob/main/Report/Presentation_project.pdf)

## Objective


## MATLAB Code 

> [!IMPORTANT]
> To run the code, you need [YALMIP](https://yalmip.github.io/) toolbox installed, and a low-level Linear Programming solver like [sedumi](https://github.com/sqlp/sedumi) or [SDTP3](https://github.com/sqlp/sdpt3).<br/>
> If you have some problem installing it, you can refer to this [presentation](https://github.com/AlePuglisi/three-tanks-distributed-control/blob/main/Report/YALMIP_HowtoInstall.pdf) provided by my professor. 

## Comments

