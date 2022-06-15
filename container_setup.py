#!/usr/bin/python3

import os
import subprocess






if __name__ == '__main__':


    result = subprocess.run(['git branch -a'], shell=True, stdout=subprocess.PIPE)
    # print('test2')
    a = result.stdout.decode('utf-8').split('\n')
    c = []
    c_print = '\nFor infrastructure-packages, what branch do you want to checkout?\n\n'
    for j, i in enumerate(a):
        if i == '':
            pass
        else:
            b = i.split('/')
            c.append(b[-1])
            c_print += f"{j}: {i}\n"
        

    value = input(c_print)
    print(c[int(value)])

    subprocess.run(f'git checkout {c[int(value)]}', shell=True)
    
    os.chdir('/root/infrastructure_ws/src/infrastructure-packages/infrastructure-arms/')

    result = subprocess.run(['git branch -a'], shell=True, stdout=subprocess.PIPE)
    # print('test2')
    a = result.stdout.decode('utf-8').split('\n')
    c = []
    c_print = '\nFor infrastructure-arms what, branch do you want to checkout?\n\n'
    for j, i in enumerate(a):
        if i == '':
            pass
        else:
            b = i.split('/')
            c.append(b[-1])
            c_print += f"{j}: {i}\n"
        

    value = input(c_print)
    subprocess.run(f'git checkout {c[int(value)]}', shell=True)

    os.chdir('/root/infrastructure_ws/src/infrastructure-packages/infrastructure-raspi/')

    result = subprocess.run(['git branch -a'], shell=True, stdout=subprocess.PIPE)
    # print('test2')
    a = result.stdout.decode('utf-8').split('\n')
    c = []
    c_print = '\nFor infrastructure-raspi, what branch do you want to checkout?\n\n'
    for j, i in enumerate(a):
        if i == '':
            pass
        else:
            b = i.split('/')
            c.append(b[-1])
            c_print += f"{j}: {i}\n"
        

    value = input(c_print)
    subprocess.run(f'git checkout {c[int(value)]}', shell=True)

    os.chdir('/root/infrastructure_ws/')
    subprocess.run('catkin_make', shell=True)
    subprocess.run('source devel/setup.bash', shell=True)
