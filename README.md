# Controller-Design-for-SCARA
Made a package to read the joint values from the Gazebo simulator, receive a reference value for the joints through a service, and publish joint efforts (continuously with high sampling rates) to make the joints move to these locations. The PD gains were tuned then to have fast convergence with minimal overshoot. 
