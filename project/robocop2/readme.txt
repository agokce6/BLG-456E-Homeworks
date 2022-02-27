## Ahmet Gökçe 	150180076
## Mehmet Karaaslan 	150180053
## Emre Güler 		040150342

--------------- Requirements ---------------
Built-in ros and gazebo libraries
python > 3.6
	python libraries:
	sudo apt install python3-pip
	pip install PyAudio (if gets error: try "sudo apt-get install portaudio19-dev")
	pip install SpeechRecognition
	

--------------- File Requirements ---------------
Copy and past "robocop2" folder to catkin_ws/src/ (or whatever your environment path is)
catkin_make
Copy and paste "models" folder to home/{user_name}/.gazebo/

--------------- How to run ---------------	
roslaunch robocop2 my_world.launch
rosrun robocop2 referee.py

--------------- Robot voice commands ---------------	
commands: 1,2,3,4,5,6,7,8
example: 1
(Robot goes to selected room which is 1.)



--------------- Map design ---------------	
_______________________________________
	|	|	|	|	|   R: robot location and distribution point
	|   4	|   3	|   2	|   1	|   
	|	|	|	|	|   1,8: rooms
					|
					|
	|	|	|	|	|
 R	|   8	|   7	|   6	|   5	|
_______|_______|_______|______|_______|
	

--------------- Notes ---------------

1- Robot takes tho product and brings it to the distribution point and waits for new command. 
   After receiving new command, the product that previously brought is sent (removed).

2- In referee.py function spawm_model(), it takes models path and accesses to "home/{user_name}/.gazebo/models".
   To get user_name, we use os.getlogin(). If it gives error, please change path manually.






	
