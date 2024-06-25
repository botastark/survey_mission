import socket
import os
def receive_file(save_as, port):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
	s.bind(('', port)) 
	s.listen(1) 
	print("Listening on port {}...".format(port)) 
	while True: #indef-ly wait for updates
		conn, addr = s.accept() 
		print("Connection from {}".format(addr)) # Receive the data 
	
		file_data = []
		while True: # Save the received data to a file 
		    data = conn.recv(1024)
		    if not data:
			break
		    file_data.append(data)
		
		file_data = b''.join(file_data).decode('utf-8')
		# Check if the file already exists
		if os.path.exists(save_as):
			print("File already exists. It will be replaced.")
		with open(save_as, 'w') as file: 
			file.write(file_data)
		conn.close()

	
if __name__ == "__main__": 
	SAVE_AS = '/home/uvify/catkin_ws/src/survey_mission/path/waypoints.txt' 
	PORT = 12345 # This should match the port used by the sender 
	receive_file(SAVE_AS, PORT)
