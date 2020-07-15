import numpy as np
import matplotlib.pyplot as plt

def read_csv(file_name):
    f = open(file_name, 'r')
    content = f.read()
    final_list = list()
    rows = content.split('\n')
    for i in range(len(rows)-1):
        final_list.append(float(rows[i]))
    return final_list

l = read_csv('rewardlog.csv')

x=[]
for i in range(0,200):
    x.append(i)
 
plt.plot(x, l)
 
plt.xlabel('step')
plt.ylabel('reward')
 
plt.show()
