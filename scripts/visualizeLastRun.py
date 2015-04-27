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

#lines = [" dfs df<NEW_LEGFEATURE legtrack2> asd as"]

leg_features = []

# New Creations
new_legfeature_regex = re.compile(r'(\d+\.\d+).*<NEW_LEGFEATURE\slegtrack(\d+)>')
del_legfeature_regex = re.compile(r'(\d+\.\d+).*<DELETE_LEGFEATURE\slegtrack(\d+)>')

for line in lines:
    new_detect = new_legfeature_regex.search(line)
    if new_detect:
        start = new_detect.group(1)
        id_ = int(new_detect.group(2))
        leg_features.insert(id_,{'start':start, 'end':0})
       
    del_detect = del_legfeature_regex.search(line)
    if del_detect:
        end = del_detect.group(1)
        id_ = int(del_detect.group(2))
        leg_features[id_]['end'] = end
        
        
## Plot the track continuity diagramm
plt.figure()
for i, feature in enumerate(leg_features):
    if(feature['end'] is not 0):
        plt.plot([feature['start'], feature['end']], [i,i])

#for line in lines:
#    if 'CREATED DUAL_TRACKER' in line:
#        print "found!"
#docs = yaml.load_all(stream)


#stream = open("~/.ros", "r")
#docs = yaml.load_all(stream)
#
#x = []
#y = []
#t = []
#
#for doc in docs:
#    if(doc is not None):
#        #print doc['twist']['twist']['linear']['x'], '\n'
#        y.append(float(doc['twist']['twist']['linear']['y']))
#        x.append(float(doc['twist']['twist']['linear']['x']))
#        t.append(float(doc['header']['stamp']['secs']) + float(doc['header']['stamp']['nsecs']) / numpy.power(10,9))
#    
#print x
#print t
#
## Norm
#t[:] = [f - t[0] for f in t]
#
## Plotting
#plt.plot(t,x)
#plt.plot(t,y)
#plt.ylabel('X')
#plt.xlabel('Time')
#plt.show()