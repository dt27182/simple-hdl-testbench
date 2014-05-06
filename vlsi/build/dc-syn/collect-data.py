import os
import sys
import re

dirName = "/home/eecs/wenyu/multithread-transform/simple-hdl-testbench/vlsi/build/dc-syn/"

#results arrays
fixedCycleTimes = []
dynamicCycleTimes = []
fixedAreas = []
dynamicAreas = []

#initialize results arrays to "n/a"
for i in range(0, 4):
  fixedCycleTimes.append([])
  dynamicCycleTimes.append([])
  fixedAreas.append([])
  dynamicAreas.append([])
  for j in range(0, 4):
    fixedCycleTimes[i].append("n/a")
    dynamicCycleTimes[i].append("n/a")
    fixedAreas[i].append("n/a")
    dynamicAreas[i].append("n/a")

#find fixed areas from reports
"""for i in range(0, 4):
  for j in range(0, i + 1):
    folderName = "reports" + str(i + 1) + str(j + 1) + str(0)
    areaReport = open(dirName + folderName + "/Cpu.mapped.area.rpt")
    for line in areaReport.readlines():
      words = re.split('\s+', line)
      if words[0] == "Total" and words[1] == "cell" and words[2] == "area:":
        fixedAreas[i][j] = float(words[3])"""


#find dynamic areas from reports
for i in range(0, 4):
  for j in range(0, 4):
    folderName = "reports" + str(i + 1) + str(j + 1) + str(1)
    areaReport = open(dirName + folderName + "/Cpu.mapped.area.rpt")
    for line in areaReport.readlines():
      words = re.split('\s+', line)
      if words[0] == "Total" and words[1] == "cell" and words[2] == "area:":
        dynamicAreas[i][j] = float(words[3])

#find fixed cycle times from reports
for i in range(0, 4):
  for j in range(0, 4):
    folderName = "reports" + str(i + 1) + str(j + 1) + str(1)
    timingReport = open(dirName + folderName + "/Cpu.mapped.timing.rpt")
    dynamicCycleTimes[i][j] = 0.0
    for line in timingReport.readlines():
      words = re.split('\s+', line)
      if words[1] == "data" and words[2] == "arrival" and words[3] == "time":
        print(-float(words[4]))
        if ((-float(words[4])) > dynamicCycleTimes[i][j]):
          dynamicCycleTimes[i][j] = -float(words[4])

#find dynamic cycle times from reports
for i in range(0, 4):
  for j in range(0, 4):
    folderName = "reports" + str(i + 1) + str(j + 1) + str(1)
    timingReport = open(dirName + folderName + "/Cpu.mapped.timing.rpt")
    dynamicCycleTimes[i][j] = 0.0
    for line in timingReport.readlines():
      words = re.split('\s+', line)
      if words[1] == "data" and words[2] == "arrival" and words[3] == "time":
        print(-float(words[4]))
        if ((-float(words[4])) > dynamicCycleTimes[i][j]):
          dynamicCycleTimes[i][j] = -float(words[4])

print(fixedCycleTimes)
print(dynamicCycleTimes)
print(fixedAreas)
print(dynamicAreas)








"""
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

print "post synthesis critical path length: " + str(post_synthesis_critical_path_length) + " ns"
print "post synthesis area: " + str(post_synthesis_total_area) + " um^2"
print "post synthesis power: " + str(post_synthesis_total_power) + " mW"
print "place and route critical path length: " + str(place_route_critical_path_length) + " ns"
print "place and route area: " + str(place_route_total_area) + " um^2"
print "place and route power: " + str(place_route_total_power) + " mW"
print "pt-pwr avg power(max): " + str(pt_pwr_avg_total_power) + " mW"
print "pt-pwr time peak power(max): " + str(pt_pwr_time_peak_power) + " mW"
print "total cell count: " + str(total_cell_count)
print "DFF cell count: " + str(DFF_cell_count)"""
