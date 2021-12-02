import sys
import os
from datetime import datetime, timedelta
from typing import List, Dict, Union

class Data_point:
	def __init__(self, line: str):
		self.line = line
		l = line.split()
		self.date_line = l[0]
		self.time_line = l[1]
		self.time_stamp = self.date_line + " " + self.time_line
		self.datetime = \
			datetime.strptime(self.time_stamp, '%Y/%m/%d %H:%M:%S.%f')
		self.latitude = l[2]
		self.longitude = l[3]
		self.height = l[4]
		self.Q = int(l[5])
		self.ns = l[6]
		self.sdn = l[7]
		self.sde = l[8]
		self.sdne = l[9]
		self.sdeu = l[10]
		self.sdun = l[11]
		self.age = l[12]
		self.ratio = l[13]

	def __str__(self):
		return self.line

	def __repr__(self):
		return self.datetime

def get_adjescent_datapoint(time_stamp: datetime,
		data_points: Dict[datetime, Data_point],
		threshold: int) -> Union[Data_point, None]:

	dps = get_Q_filtered_datapoints(time_stamp, data_points, 1, threshold)
	if len(dps) > 0:
		# Found some data with rtk-fix ( sub cm accurracy (GOOD))
		return get_closest_datapoint(time_stamp, dps, threshold)

	dps = get_Q_filtered_datapoints(time_stamp, data_points, 2, threshold)
	if len(dps) > 0:
		# Found some data with rtk-float ( 1cm < accuracy < 1m )
		return get_closest_datapoint(time_stamp, dps, threshold)


	dps = get_Q_filtered_datapoints(time_stamp, data_points, 5, threshold)
	if len(dps) > 0:
		# Found some data with rtk-single ( accuracy > 1m (BAD))
		return get_closest_datapoint(time_stamp, dps, threshold)

	# No adjescent data found,
	# only happens with no data or
	# data loss over more than +- threshold seconds
	return None

def get_Q_filtered_datapoints(time_stamp: datetime,
		data_points: Dict[datetime, Data_point], q: int,
		max_threshold: int) -> Dict[datetime, Data_point]:

	d_seconds = 1
	dps = {}

	while d_seconds <= max_threshold:
		d_time = timedelta(seconds=d_seconds)
		d_pos = time_stamp + d_time
		d_neg = time_stamp - d_time

		# Check positive time
		if d_pos in data_points:
			if data_points[d_pos].Q == q:
				dps[d_pos] = data_points[d_pos]
		# Check negative time
		if d_neg in data_points:
			if data_points[d_neg].Q == q:
				dps[d_neg] = data_points[d_neg]

		# Increment search time
		d_seconds += 1

	return dps

def get_closest_datapoint(time_stamp: datetime,
		data_points: Dict[datetime, Data_point],
 		threshold: int) -> Union[Data_point, None]:

	d_seconds = 1

	while d_seconds <= threshold:

		d_time = timedelta(seconds=d_seconds)
		d_pos = time_stamp + d_time
		d_neg = time_stamp - d_time

		if d_pos in data_points: # Prioritize positive timestamps
			return data_points[d_pos]
		elif d_neg in data_points:
			return data_points[d_neg]
		else:
			d_seconds += 1
	return None


if __name__ == "__main__":
	# Check argc
	if (len(sys.argv) < 3):
		print("Too few input arguments")
		print("pickPositions.py <FILE.pos> <FILE.txt>")
		sys.exit("-1")
	elif (len(sys.argv) > 3):
		print("Too many input arguments")
		print("pickPositions.py <FILE.pos> <FILE.txt>")
		sys.exit(-1)

	# Check if input args are correct filename
	if(not sys.argv[1].endswith((".pos", ".POS"))):
		print("File: " + sys.argv[1] + ", not .pos format!")
		sys.exit(-1)
	elif(not sys.argv[2].endswith((".txt", ".TXT"))):
		print("File: " + sys.argv[2] + ", not .txt format!")
		sys.exit(-1)


	# Check if input args exists
	if(not os.path.isfile(sys.argv[1])):
		print("File: " + sys.argv[1] + ", Does not exist!")
		sys.exit(-1)
	elif(not os.path.isfile(sys.argv[2])):
		print("File: " + sys.argv[2] + ", Does not exist!")
		sys.exit(-1)

	# Add print help?
	threshold = 5
	pos_file = sys.argv[1]
	time_file = sys.argv[2]
	dir = os.path.dirname(pos_file)
	filtered_pos_file = \
		os.path.join(dir, "filtered_" + os.path.basename(pos_file))


	header, time_stamps = [], []
	found_time_stamps = []

	data_points = {}

	# Copy header and create all objects
	with open(pos_file, "r") as p_file:
		while (line := p_file.readline().rstrip()):
			if line.startswith('%'):
				if line.startswith('%%'): # Only for testing
					continue
				header.append(line + '\n')
			elif line != '':
				dp = Data_point(line)
				#data_points.append(dp)
				data_points[dp.datetime] = dp

	# read timestamps
	with open(time_file, "r") as f_time:
		print("Time stamps for data points: ")
		while (time := f_time.readline().rstrip()):
			time_stamps.append(datetime.strptime(time, '%Y/%m/%d %H:%M:%S.%f'))

	print(*time_stamps, sep='\n')

	# Filter time stamps for data
	dps = list(data_points.values())
	found_time_stamps = \
		list(filter(lambda x: x.datetime in time_stamps, dps))

	# Write copy header to new file
	with open(filtered_pos_file, "w") as f_pos_file:
		for line in header:
			f_pos_file.write(line)

	if len(found_time_stamps) == len(time_stamps):
		print("\nAll timestamps found in pos-file.")

		with open(filtered_pos_file, "a") as f_pos_file:
			for key in found_time_stamps:
				f_pos_file.write("{}\n".format(key.line))

	else:
		print("\nAll timestamps where not found in the positioning data.")
		print("Checking for other data.")

		# Get the time stamps that are not in the original files
		missing_time_stamps = \
			list(filter(lambda x: x not in \
				[dp.datetime for dp in dps], time_stamps))

		print("\nMISSING ", len(missing_time_stamps), " TIME STAMPS.")
		print(*missing_time_stamps, sep='\n')

		errors = []
		for x in missing_time_stamps:
			adj_x = get_adjescent_datapoint(x, data_points, threshold)
			if adj_x:
				found_time_stamps.append(adj_x)
				print("\nFOUND REPLACEMENT FOR: ")
				print(x)
				print(adj_x)
			else:
				# NOTHING FOUND, SOMETHING IS WRONG
				errors.append(x)

		if len(errors) > 0:
			error_file = os.path.join(dir, "errors.txt")
			with open(error_file, mode='w') as error_f:
				error_f.write("The following time stamps had no data " \
						"within +-{} threshold seconds.\n".format(threshold))
				for e in errors:
					error_f.write("{}\n".format(e))


		# Sort and write data
		found_time_stamps.sort(key=lambda x: x.datetime)
		with open(filtered_pos_file, "a") as f_pos_file:
			for key	in found_time_stamps:
				f_pos_file.write("{}\n".format(key.line))
