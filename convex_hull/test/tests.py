import os
import subprocess
from subprocess import Popen, PIPE, STDOUT
import time

convex_hull_app_ = \
    ("C:/projects/parallel/convex_hull/build/Release/convex_hull.exe" \
    if (os.name == "nt") else \
        "/home/wf34/projects/parallel/convex_hull/build/convex_hull")
convex_hull_gs_app_ = \
    ("C:/projects/parallel/convex_hull/build/Release/convex_hull_gs.exe" \
    if (os.name == "nt") else \
        "/home/wf34/projects/parallel/convex_hull/build/convex_hull_gs")
normal_rbox_app_ = \
    ("C:/projects/parallel/convex_hull/build/Release/normal_rbox.exe" \
    if (os.name == "nt") else \
        "/home/wf34/projects/parallel/convex_hull/build/normal_rbox")
allowed_error_ = float (1e-9)

qhull_time_ = 0
chull_time_ = 0

def deserialize_cycle (serialized_cycle):
    lines = serialized_cycle.split("\n")
    cycle = []
    for line in lines:
        if (len (line) < 5):
            continue
        choordinates = line.split(" ")
        cycle.append ((float (choordinates[0]), float (choordinates[1])))
    cycle = sorted (cycle, key = lambda choord : choord[0])
    # for x in cycle:
    #     print x
    return cycle



def compare_cycles (cycle_string_first, cycle_string_second):
    cycle_first = deserialize_cycle (cycle_string_first)
    # print "/////////////////"
    cycle_second = deserialize_cycle (cycle_string_second)

    if (len (cycle_first) != len (cycle_second)):
        return False
    for i in range (len (cycle_first)):
        if (abs (cycle_first[i][0] - cycle_second[i][0]) > allowed_error_ or \
           abs (cycle_first[i][1] - cycle_second[i][1]) > allowed_error_):
            return False

    return True


def hull_test (seed, problem_size):
    global qhull_time_, chull_time_
    seeding_argument = 't' + str(seed)
    # print "seed was", seeding_argument
    # rbox_process = Popen (['rbox', '1024', 'D2', seeding_argument], stdout = PIPE, shell = True)
    rbox_process = Popen ([normal_rbox_app_, str (problem_size),
                           'D2', seeding_argument], stdout = PIPE)
    problem = rbox_process.communicate ()
    # print "==========\n", problem[0], "==========\n"

    q_tic = time.clock()
    qh = subprocess.Popen(['qhull', 'p'],
                          stdout=PIPE, stdin=PIPE, stderr=STDOUT)
    qhull_output = qh.communicate (input=problem[0])[0]
    q_toc = time.clock()
    qhull_time_ += q_toc - q_tic
    # print "Qhull Result:\n", qhull_output
    
    c_tic = time.clock()
    ch = subprocess.Popen([convex_hull_app_],
                          stdout=PIPE, stdin=PIPE, stderr=STDOUT)
    convex_hull_output = ch.communicate (input=problem[0])[0]
    c_toc = time.clock()
    chull_time_ += c_toc - c_tic
    # print "Our Result:\n", convex_hull_output

    if (compare_cycles (qhull_output, convex_hull_output)):
        # print "test passed"
        return True
    else:
        print "test failed"
        return False


def is_enet_valid (set, enet):
    center = []
    return False



def check_enet (seed):
    seeding_argument = 't' + str(seed)
    rbox_process = Popen (['rbox', '64', 'D2', seeding_argument], stdout = PIPE)
    rbox_output = rbox_process.communicate ()[0]
    # print "==========\n", rbox_output, "==========\n"

    ch = subprocess.Popen([convex_hull_app_, 'p'], stdout=PIPE, stdin=PIPE)
    convex_hull_output = ch.communicate (input=rbox_output)[0]
    # print "Our Result:\n", convex_hull_output, "==========\n"
    enet = deserialize_cycle (convex_hull_output)

    first_line_end = rbox_output.find ('\n')
    rbox_data = rbox_output [first_line_end + 1:]
    # print "==========\n", rbox_data, "==========\n"
    whole_pointset = deserialize_cycle (rbox_data)
    # print "whole_pointset ==========="
    # for x in range (len (whole_pointset)):
    #     print whole_pointset[x][0], " ", whole_pointset[x][1]
    # print "enet ==========="
    # for x in range (len (enet)):
    #     print enet[x][0], " ", enet[x][1]
    if not is_enet_valid (whole_pointset, enet):
        print "e-net test failed"



def testing (problem_size):
    print "for size ", problem_size
    tests_amount = 2
    random_seeds = [350, 1815]
    for i in range (8):
        random_seeds.append (random_seeds[-1] + random_seeds[-2])

    for x in range (10):
        # print "*** TEST", x, "**********"
        if not hull_test (random_seeds[x], problem_size):
            break
    print "qhull impl running time", qhull_time_ / 10.0
    print "chull impl running time", chull_time_ / 10.0


def testing_for_performance ():
    problem_sizes = [1024, 2048, 4096, 8192, 16384, \
                     32768, 65536, 131072, 262144]
    # problem_sizes = [524288,1048576,2097152]
    for x in problem_sizes:
        testing (x)


if __name__ == "__main__":
    testing_for_performance ()
