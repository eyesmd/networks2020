import json
from more_itertools import ilen
import os

# Json spec
#{
#  "instance_name": str,
#  "graph": {
#    "vertex_count": int,
#    "arcs": [
#      {
#        "tail": int,
#        "head": int,
#        "demand": int,
#        "travel_time": [
#          {
#            "piece_end": float,
#            "speed": float
#          }
#        ],
#      }
#    ],
#    "edges": [
#      {
#        "tail": int,
#        "head": int,
#        "demand": int,
#        "travel_time": [
#          {
#            "piece_end": float,
#            "speed": float
#          }
#        ],
#      }
#    ]
#  },
#  "vehicle_count": int,
#  "capacity": int,
#  "depot": int,
#  "horizon": [
#    int,
#    int
#  ],
#  "service speed factor": float
#}

def convert(input_path, output_path):
	with open(input_path) as file:
		# Read headers
		get_value = lambda file : file.readline().split(" : ")[1].rstrip("\n")
		instance_name = get_value(file)
		vertex_count = int(get_value(file))
		required_edge_count = int(get_value(file))
		nonrequired_edge_count = int(get_value(file))
		vehicle_count = int(get_value(file))
		capacity = int(get_value(file))
		depot = int(get_value(file))
		start_time = int(get_value(file))
		end_time = int(get_value(file))
		service_speed_factor = float(get_value(file))
		# Read graph
		edges = []
		file.readline() # skip [ NETWORK_DATA]
		for line in file.readlines():
			edge = {}
			symbols = line.split()
			edge["tail"] = int(symbols[0])
			edge["head"] = int(symbols[1])
			edge["distance"] = int(symbols[2])
			edge["demand"] = int(symbols[3])
			edge["period_count"] = int(symbols[4])
			edge["period_ends"] = []
			current_symbol = 5
			current_symbol += 1 # skip '['
			for i in range(edge["period_count"] - 1):
				edge["period_ends"].append(int(symbols[current_symbol]))
				current_symbol += 1
			current_symbol += 1 # skip ']'
			edge["period_speeds"] = []
			current_symbol += 1 # skip '['
			for i in range(edge["period_count"]):
				edge["period_speeds"].append(float(symbols[current_symbol]))
				current_symbol += 1
			edges.append(edge)
		# Write headers
		output = {}
		output["instance_name"] = instance_name
		output["vehicle_count"] = vehicle_count
		output["capacity"] = capacity
		output["depot"] = depot
		output["horizon"] = [ start_time, end_time ]
		output["service_speed_factor"] = service_speed_factor
		# Write graph
		graph = {}
		graph["vertex_count"] = vertex_count
		graph["arcs"] = []
		graph["edges"] = []
		for e in edges:
			assert(e["head"] != e["tail"] for eo in edges)
			assert(e["head"] < vertex_count and e["tail"] < vertex_count)
			if sum(1 for eo in edges if eo["head"] == e["head"] and eo["tail"] == e["tail"] ) != 1:
				raise Exception("Repeated edge " + str(e["tail"]) + ' -> ' + str(e["head"]))

			is_edge = any(eo["tail"] == e["head"] and eo["head"] == e["tail"] for eo in edges)

			#if is_edge and e["tail"] > e["head"]:
			#	continue

			out_e = {}
			out_e["tail"] = e["tail"]
			out_e["head"] = e["head"]
			out_e["distance"] = e["distance"]
			out_e["demand"] = e["demand"]
			out_e["travel_time"] = []
			for i in range(e["period_count"]):
				new_piece = {
					"piece_end": e["period_ends"][i] if i < e["period_count"] - 1 else end_time,
					"speed": e["period_speeds"][i]					
				}
				out_e["travel_time"].append(new_piece)

			if is_edge:
				graph["edges"].append(out_e)
			else:
				graph["arcs"].append(out_e)
			output["graph"] = graph
		# Write to file
		with open(output_path, 'w') as output_file:
			json.dump(output, output_file, indent=4)
		# Return instance name
		return instance_name

index_entries = []

def convert_dir(dir_path, prefix):
	for input_name in os.listdir(dir_path):
		output_name = prefix + "_" + os.path.splitext(input_name)[0] + ".json"
		instance_name = convert(dir_path + "/" + input_name, "output/" + output_name)
		index_entry = {
			"file_name": output_name,
			"instance_name": instance_name,
			"tags": [ prefix ]
		}
		index_entries.append(index_entry)

for f in os.listdir("output"):
    os.remove(os.path.join("output", f))
convert_dir("input/Type_H", "H")
convert_dir("input/Type_M", "M")
convert_dir("input/Type_L", "L")
with open("output/index.json", 'w') as output_file:
	json.dump(index_entries, output_file, indent=4)


#[{
#	"file_name": "C101_25.json",
#	"instance_name": "C101_25",
#	"tags": ["C101_25", "N25", "TC1", "C01", "NOTWS"]
#}, {