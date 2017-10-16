import os

plan_time = os.getenv("HOME") + '/.ros/plan.time'

def FileChanged(filename):
    #check file existance
    if os.path.isfile(plan_time):
        print "."
    else:
        print "warning : file plan.time does not exist, will create one for you"
        time_file = open(plan_time, 'w')
        time_file.write("0.0")
        time_file.close()
    time_file = open(plan_time, 'r')
    file_time = time_file.readline()
    file_creation_time = os.stat(filename).st_mtime
    time_file = open(plan_time, 'w')
    time_file.write(str(file_creation_time))
    time_file.close()
    if abs(float(file_time) - float(file_creation_time)) < 0.05:
        return False
    else:
        return True
    
def test_this_script():
    filename = 'test_file.txt'
    if FileChanged(filename):
        print "file has changed"
    else:
        print "file did not change"
    
if __name__ == "__main__":
    test_this_script()