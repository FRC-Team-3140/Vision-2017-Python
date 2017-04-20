@echo off
cd C:\Users\far5guest\Documents\Vision-2017-Python
set /P input= Enter desired input filename:
set /P output= Enter desired output filename:
python visionnew.py --ifile %input% --ofile %output% --debug


