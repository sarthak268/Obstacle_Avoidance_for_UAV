def convert(deg,min,sec):
	mindeg = 0.0166667*min
	secdeg = 0.00027777833333*sec
	deg = mindeg + secdeg + deg
	print (deg)

deg = input ("Enter degrees : ")
mindeg = input("Enter Minutes of Arc : ")
secdeg = input("Enter Seconds of Arc : ")

convert(deg,mindeg,secdeg)

