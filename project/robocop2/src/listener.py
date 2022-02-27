## Ahmet Gökçe 	150180076
## Mehmet Karaaslan 	150180053
## Emre Güler 		040150342


import speech_recognition as sr

from threading import Thread

class Listener():
	room = None
	stop = False
    
	def __init__(self):
		self.recognizer = sr.Recognizer()
		self.microphone = sr.Microphone()
        
		self.recognizer.pause_threshold = 0.5
		with self.microphone as source: 
			self.recognizer.adjust_for_ambient_noise(source, duration=0.2)

		self.rooms = ["1","2","3","4","5","6","7","8"]
    		
	def start(self):
		thread = Thread(target=self.run)
		thread.start()
    
	def run(self):   
		while True:
			while not self.stop:
				try:
					with self.microphone as source:
						print("listening")
						audio = self.recognizer.listen(source)

						print("getting text")
						text = self.recognizer.recognize_google(audio)
						text = text.lower()

						print("I have listened:", text)
						
						if(text == "close"):
							print('closing program.')
							break	
                	
						elif(text in self.rooms):
							self.room = text
							print("going to room", text)
						else:
							print("Unknown command")
                	
				except sr.UnknownValueError:
					print("Value error")
					self.room = None
					self.recognizer = sr.Recognizer()



if __name__ == "__main__":
   l = Listener()
   l.start()
   #main()
