import serial 
import time 

PORT = "/dev/ttyACM0"
BAUD = 115200

def safe_float(value):
	try:
		if value.lower() =="nan":
			return None
			return float(value) 
	except:
			return None 
	
def safe_int(value):
	try:
		return int(value)
	except:
		return None
		
def parse_line(line):
	parts=line.strip().split(",")
	
	if len(parts)!=9:
		return None
		
	data = {
	"rain":safe_int(parts[0]),
	"sound":safe_int(parts[1]),
	"temp":safe_float(parts[2]),
	"humidity": safe_float (parts[3]),
	"pressure": safe_float(parts[4]),
	"vibration":safe_float(parts[5]),
	"distance":safe_float(parts[6]),
	"hazardCode":safe_int(parts[7]),
	"hazardText":parts [8]
	}
	return data
	
def main():
	print(f"Opening Serial port {PORT} at {BAUD} baud ...")
	ser = serial.Serial(PORT,BAUD, timeout=1)
	time.sleep(2)
	
	print("Connected. Waiting for ARduino data ...\n")
	
	while True:
		try:
			line=ser.readline().decode("utf-8",errors="ignore").strip()
			if not line:
				continue
			if line.startswith("rain,sound,temp,humidity"):
				print("Header recieved.")
				continue
				
			data=parse_line(line)
			
			if data is None:
				print("Invalid line:",line)
				continue
			with open("hazard_data_log.csv","a") as f:
                f.write(line + "\n")
			print("-----Hazard Node Reading-----")
			-print(f"Rain Value : {data['rain']}")
			print(f"Sound Value : {data['sound']}")
			print(f"Temperature(C) : {data['temp']}")
			print(f"Humidity(%) : {data['humidity']}")
			print(f"Pressure(hPa) : {data['pressure']}")
			print(f"Vibration Level : {data['vibration']}")
			print(f"Distance(cm) : {data['distance']}")
			print(f"Hazard Code : {data['hazardCode']}")
			print(f"Hazard Text : {data['hazardText']}")
			print()
			
		except KeyboardInterrupt:
				print("\nStopped by user.")
				break
		except Exception as e:
				print("Error:",e)
				time.sleep(1)
				
	if __name__=="__main__":
		main()
		
