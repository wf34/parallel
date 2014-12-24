import subprocess
from subprocess import Popen, PIPE, STDOUT

convex_hull_app = "C:/projects/parallel/convex_hull/build/Release/convex_hull.exe"


def hull_test (seed):
    seeding_argument = 't' + str(seed)
    rbox_process = Popen (['rbox', '10', 'D2', seeding_argument], stdout = PIPE)
    problem = rbox_process.communicate ()
    print "==========\n", problem[0], "==========\n"

    qh = subprocess.Popen(['qhull', 'p'], stdout=PIPE, stdin=PIPE, stderr=STDOUT)
    qhull_output = qh.communicate (input=problem[0])[0]
    print "Qhull Result:\n", qhull_output

    ch = subprocess.Popen([convex_hull_app, 'p'], stdout=PIPE, stdin=PIPE, stderr=STDOUT)
    convex_hull_output = ch.communicate (input=problem[0])[0]
    print "Our Result:\n", convex_hull_output

    if (qhull_output == convex_hull_output):
        print "test passed"
    else:
        print "test failed"



def testing ():
    # tests_amount = 2
    # random_seeds = [350, 1815]
    # for x in range (0, 2):
    #     print "*** TEST", x, "**********"
    #     hull_test (random_seeds[x])
    hull_test (1815)



if __name__ == "__main__":
    testing ()
