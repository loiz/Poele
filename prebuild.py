import datetime
import sys
import os
Import("env")
FILENAME_VERSION_H = 'src/version.h'

build_no = 0
env.Dump()

hf = """
#ifndef VERSION
  #define VERSION "{}{}{}{}{}{}"
#endif
""".format(datetime.datetime.now().year,datetime.datetime.now().month,datetime.datetime.now().day,datetime.datetime.now().hour,datetime.datetime.now().minute,datetime.datetime.now().second)
with open(FILENAME_VERSION_H, 'w+') as f:
    f.write(hf)

for path in os.listdir("data"):
  print("Generation de : " + path)
  first = True
  with open("src/"+path.replace(".","_") + ".h",'wb') as result_file:
    result_file.write(b'const char %s[] = {' % path.replace(".","_").encode('utf-8'))
    for b in open("data/" + path, 'rb').read():
      if first:
        result_file.write(b'0x%02X' % b)
        first = False
      else:
        result_file.write(b',0x%02X' % b)
    result_file.write(b'};')
  result_file.close()