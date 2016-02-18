import yaml
import numpy
import matplotlib.pyplot as plt
import os, os.path
import re

def all_subdirs_of(b='.'):
  result = []
  for d in os.listdir(b):
    bd = os.path.join(b, d)
    if os.path.isdir(bd): result.append(bd)
  return result

path_to_log = os.environ['HOME'] + "/.ros/log/"
all_subdirs = all_subdirs_of(path_to_log)
latest_subdir = max(all_subdirs, key=os.path.getmtime)

print latest_subdir

stream = open(latest_subdir + "/rosout.log", "r")
content = stream.read()

runs = content.split('CREATED DUAL_TRACKER')

# Get the latest run
current_run = runs[-1]

lines = current_run.split('\n')


# Close all
plt.close('all')
                

#lines = [" dfs df<NEW_LEGFEATURE legtrack2> asd as"]

leg_features = []

print "Starting to parse the content!"

# New Creations
new_legfeature_regex = re.compile(r'(\d+\.\d+).*<NEW_LEGFEATURE\slegtrack(\d+)>')
del_legfeature_regex = re.compile(r'(\d+\.\d+).*<DELETE_LEGFEATURE\slegtrack(\d+)>')

for line in lines:
    new_detect = new_legfeature_regex.search(line)
    if new_detect:
        start = float(new_detect.group(1))
        id_ = int(new_detect.group(2))
        leg_features.insert(id_,{'id':id_,'start':start, 'end':0})
       
    else:
        del_detect = del_legfeature_regex.search(line)
        if del_detect:
            end = float(del_detect.group(1))
            id_ = int(del_detect.group(2))
            leg_features[id_]['end'] = end
        
# Find the maximum time
min_time = min([x['start'] for x in leg_features])
max_time = max([x['end'] for x in leg_features])
      
## Plot the track continuity diagramm
print "Plotting!"
plt.figure()
for i, feature in enumerate(leg_features):
    if(feature['end'] is not 0):
        plt.plot([feature['start'], feature['end']], [i,i], marker='.', c='b')
    else:
        plt.plot([feature['start'], feature['start']], [i,i], marker='.', c='r') # The first dot
        plt.plot([feature['start'], max_time], [i,i], marker='', c='r') # The line
        plt.plot([max_time, max_time+(max_time-min_time)*0.1], [i,i], marker='', c='r', ls=':') # The dottet line at the end
        
plt.xlabel('Time')
plt.ylabel('Tracker')
plt.grid(True)
        
plt.show()