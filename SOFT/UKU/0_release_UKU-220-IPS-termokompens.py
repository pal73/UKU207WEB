#!/usr/bin/env python

import re
import os
import shutil
import sys

#targetDir = "P:\\ПТЭСТ\\ИПС_ЗВУ\\"
targetDir = "P:\\Прошивки2018\\ИПС_ЗВУ\\"
source_file = 'curr_version.c'
source_file1 = ".\\EXE\\UKU_220_IPS_TERMOKOMPENSAT.hex"

version_string = 'const short HARDVARE_VERSION = {};'
subversion_string = 'const short SOFT_VERSION = {};'
build_string = 'const short BUILD = {};'
build_year_string = 'const short BUILD_YEAR = {};'
build_month_string = 'const short BUILD_MONTH = {};'
build_day_string = 'const short BUILD_DAY = {};'

version_re = re.compile(version_string.format('(\d+)'))
subversion_re = re.compile(subversion_string.format('(\d+)'))
build_re = re.compile(build_string.format('(\d+)'))
build_year_re = re.compile(build_year_string.format('(\d+)'))
build_month_re = re.compile(build_month_string.format('(\d+)'))
build_day_re = re.compile(build_day_string.format('(\d+)'))


with open(source_file) as f:
    for line in f:
        match = version_re.search(line)
        if match:
            version = match.group(1)
        match = subversion_re.search(line)
        if match:
            subversion = match.group(1)
        match = build_re.search(line)
        if match:
            build = match.group(1)
        match = build_year_re.search(line)
        if match:
            build_year = match.group(1)
        match = build_month_re.search(line)
        if match:
            build_month = match.group(1)
            mont=int(build_month)
            build_month = "%02d" % mont
        match = build_day_re.search(line)
        if match:
            build_day = match.group(1)
            d=int(build_day)
            build_day= "%02d" % d

targetDir_=targetDir+build_year+"_"+build_month+"_"+build_day+"__"+version+"."+subversion+"."+build
print(targetDir_)

try : os.mkdir(targetDir_)
except FileExistsError :
    print ("Папка уже существует")
    input()
    #os.system("pause")

target_file=targetDir_+ "\\ips_zvu_"+"_"+build_year+"_"+build_month+"_"+build_day+"__"+version+"."+subversion+"."+build+".hex"

try : shutil.copy(source_file1,target_file)
except Exception as err:
    print(err)

messageStr = input("Если хотите оставить комментарий в файле истории, \n оставьте его здесь - ")
if messageStr != '':
    print (messageStr)
    my_file = open(targetDir+ "\\history.txt", 'a')
    my_file.write("\n \n \n" +build_year+"/"+build_month+"/"+build_day+"   "+version+"."+subversion+"."+build)
    my_file.write("\n \n" + messageStr)
    my_file.close()

    
"""
for param in sys.argv:
    print (param)
"""    
"""
targetDir1 = targetDir+"2018_05_30"
shutil.copy("test.hex",targetDir1+"\\efert"+str(version)+"."+str(subversion)+"."+str(build)+".hex")
"""

