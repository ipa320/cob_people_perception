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

people_trackers = []
print "Starting to parse the content!"

# New Creations
new_peopletracker_regex = re.compile(r'(\d+\.\d+).*<NEW_PEOPLETRACKER\s(\d+)-(\d+)>')
del_peopletracker_regex = re.compile(r'(\d+\.\d+).*<DELETE_PEOPLETRACKER\s(\d+)-(\d+)>')

for line in lines:
    new_detect = new_peopletracker_regex.search(line)
    if new_detect:
        start = float(new_detect.group(1))
        id_ = [None,None]
        id_[0] = int(new_detect.group(2))
        id_[1] = int(new_detect.group(3))
        people_trackers.append({'id':id_,'start':start, 'end':0})
        #print "found new tracker", id_
       
    else:
        del_detect = del_peopletracker_regex.search(line)
        if del_detect:
            end = float(del_detect.group(1))
            id_ = [None,None]
            id_[0] = int(del_detect.group(2))
            id_[1] = int(del_detect.group(3))
            
            for idx, people_tracker in enumerate(people_trackers):
                if(people_tracker['id'] == id_):
                    #print id_, "deleted"
                    people_tracker['end'] = end
                    people_trackers[idx] = people_tracker
                    
                    
            #people_trackers[id_]['end'] = end
            
print people_trackers
        
# Find the maximum time
min_time = min([x['start'] for x in people_trackers])
max_time = max([x['end'] for x in people_trackers])
      
## Plot the track continuity diagramm
print "Plotting!"
plt.figure()
for i, feature in enumerate(people_trackers):
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