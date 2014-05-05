import os
import sys
import re
dir = sys.argv[1]
module_name = sys.argv[2]

post_synthesis_timing = open("/scratch/cs250-ao/" + dir + "/build/dc-syn/current-dc/reports/" + module_name +".mapped.timing.rpt")
post_synthesis_critical_path_length = 0.0
for line in post_synthesis_timing.readlines():
  words = re.split('\s+', line)
  if words[1] == "data" and words[2] == "arrival" and words[3] == "time":
    post_synthesis_critical_path_length = -float(words[4])

post_synthesis_area = open("/scratch/cs250-ao/" + dir + "/build/dc-syn/current-dc/reports/" + module_name +".mapped.area.rpt")
post_synthesis_total_area = 0.0
for line in post_synthesis_area.readlines():
  words = re.split('\s+', line)
  if words[0] == "Total" and words[1] == "cell" and words[2] == "area:":
    post_synthesis_total_area = float(words[3])

post_synthesis_power = open("/scratch/cs250-ao/" + dir + "/build/dc-syn/current-dc/reports/" + module_name +".mapped.power.rpt")
post_synthesis_total_power = 0.0
for line in post_synthesis_power.readlines():
  words = re.split('\s+', line)
  if words[0] == module_name:
    post_synthesis_total_power = float(words[4]) / 1000

place_route_timing = open("/scratch/cs250-ao/" + dir + "/build/icc-par/current-icc/reports/chip_finish_icc.timing.rpt")
place_route_critical_path_length = 0.0
for line in place_route_timing.readlines():
  words = re.split('\s+', line)
  if words[1] == "data" and words[2] == "arrival" and words[3] == "time":
    place_route_critical_path_length = -float(words[4])

place_route_area = open("/scratch/cs250-ao/" + dir + "/build/icc-par/current-icc/reports/chip_finish_icc.area.rpt")
place_route_total_area = 0.0
for line in place_route_area.readlines():
  words = re.split('\s+', line)
  if words[0] == "Total" and words[1] == "cell" and words[2] == "area:":
    place_route_total_area = float(words[3])

place_route_power = open("/scratch/cs250-ao/" + dir + "/build/icc-par/current-icc/reports/chip_finish_icc.power.rpt")
place_route_total_power = 0.0
for line in place_route_power.readlines():
  words = re.split('\s+', line)
  if words[0] == module_name and len(words) > 4:
    place_route_total_power = float(words[4]) / 1000

pt_pwr_avg_power = open("/scratch/cs250-ao/" + dir + "/build/pt-pwr/current-pt/reports/vcdplus.power.avg.max.report")
pt_pwr_avg_total_power = 0.0
for line in pt_pwr_avg_power.readlines():
  words = re.split('\s+', line)
  if words[0] == module_name and len(words) > 4:
    pt_pwr_avg_total_power = float(words[4]) * 1000

pt_pwr_time_power = open("/scratch/cs250-ao/" + dir + "/build/pt-pwr/current-pt/reports/vcdplus.power.time.max.report")
pt_pwr_time_peak_power = 0.0
for line in pt_pwr_time_power.readlines():
  words = re.split('\s+', line)
  if words[0] == module_name and len(words) > 4:
    pt_pwr_time_peak_power = float(words[4]) * 1000

post_synthesis_hierarchy = open("/scratch/cs250-ao/" + dir + "/build/dc-syn/current-dc/reports/" + module_name +".mapped.reference.rpt")
total_cell_count = 0
DFF_cell_count = 0
in_cell_section = False
for line in post_synthesis_hierarchy.readlines():
  words = re.split('\s+', line)
  if "--" in words[0] and not in_cell_section:
    in_cell_section = True
  elif "--" in words[0] and in_cell_section:
    in_cell_section = False
  elif in_cell_section and words[0] != "gcdGCDUnitCtrl" and words[0] != "gcdGCDUnitDpath_W16" and words[0] != "gcdGCDUnitDpath_W32" and not "SNPS" in words[0]:
    total_cell_count = total_cell_count + int(words[3])
    if re.match(r'DFF*', words[0]):
      DFF_cell_count = DFF_cell_count + int(words[3])

print "post synthesis critical path length: " + str(post_synthesis_critical_path_length) + " ns"
print "post synthesis area: " + str(post_synthesis_total_area) + " um^2"
print "post synthesis power: " + str(post_synthesis_total_power) + " mW"
print "place and route critical path length: " + str(place_route_critical_path_length) + " ns"
print "place and route area: " + str(place_route_total_area) + " um^2"
print "place and route power: " + str(place_route_total_power) + " mW"
print "pt-pwr avg power(max): " + str(pt_pwr_avg_total_power) + " mW"
print "pt-pwr time peak power(max): " + str(pt_pwr_time_peak_power) + " mW"
print "total cell count: " + str(total_cell_count)
print "DFF cell count: " + str(DFF_cell_count)
