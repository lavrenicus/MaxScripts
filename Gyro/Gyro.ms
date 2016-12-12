--DON'T LAUNCH!! NEED ADD MINUS VALUE N-P

rollout Gyro	"Gyro" width:200 height:200
(
	button ConOpen "Open Conection" pos:[11,10] width:100 height:20
	button Object "Object" pos:[120,10] width:60 height:20
	button ConClose "Close connection" pos:[10,40] width:100 height:20
	button CapStart "Start Capture" pos:[10,70] width:100 height:20
	button CapStop "Stop Capture" pos:[10,100] width:100 height:20
	
	
	on ConOpen pressed  do
	(
		global port = dotNetObject "System.IO.Ports.SerialPort" 
		port.portname = "COM4"
		port.baudrate = 115200
		port.parity = port.parity.none
		port.databits = 8
		port.stopbits = port.stopbits.one
		port.handshake= port.handshake.RequestToSend
		port.Open()
		if(port.IsOpen) then
			(
				messageBox ("Port opened")
			)
			else
			(
				messageBox ("Can not open port ")
			)
	)
	on Object pressed do
	(
		if($!=undefined) then
		(
			global GyrObj = $
			print(GyrObj)
		)
		else
		(
			messagebox("Selection is empty")
		)
	)
	on ConClose pressed  do
	(
		if (capturing!=1) do
		Port.Close()
	)
	on CapStart pressed  do
	(
		Global Capturing = 1
		print(GyrObj)
		if(GyrObj!=undefined) then
		(
			Global theTimer = dotNetObject "System.Windows.Forms.Timer" --create a Timer
			fn GetAxis = 
			(
				for i=1 to 3 do
				(
					port.WriteLine(i as string)
					global Xres
					global Yres
					global Zres
					global Incom = port.readLine()
					print(Incom)
					/*
					for i=1 to Incom.count do
					(
						print[i]
						if(Incom[i]=='X') then 
						(
							Xres = (Incom[i+1] as integer*100) +  (Incom[i+2] as integer*10) +  (Incom[i+3] as integer)
						)
						if(Incom[i]=='Y') then 
						(
							Yres = (Incom[i+1] as integer*100) +  (Incom[i+2] as integer*10) +  (Incom[i+3] as integer)
						)
						if(Incom[i]=='Z') then 
						(
							Zres = (Incom[i+1] as integer*100) +  (Incom[i+2] as integer*10) +  (Incom[i+3] as integer)
						)
					)
					*/
					Xres = (Incom[2] as integer*100) +  (Incom[3] as integer*10) +  (Incom[4] as integer)
					Yres = (Incom[6] as integer*100) +  (Incom[7] as integer*10) +  (Incom[8] as integer)
					Zres =(Incom[10] as integer*100) + (Incom[11] as integer*10) + (Incom[12] as integer)
					GyrObj[i].rotation.x_rotation = Xres
					GyrObj[i].rotation.y_rotation = Yres
					GyrObj[i].rotation.z_rotation = Zres
					--GyrObj[i].rotation = (eulerAngles Xres Yres Zres)as quat
					Sleep 0.1
				)
			)
		dotnet.addEventHandler theTimer "tick" GetAxis --add ON TICK event hander to call the function
		theTimer.interval = 100
		theTimer.start() --start the Timer
		--theTimer.stop()
		)
		else
		(
			messagebox("Select obj and press Object")
		)
	)
	on CapStop pressed  do
	(
		theTimer.Stop()
		capturing = 0
	)
)

CreateDialog Gyro