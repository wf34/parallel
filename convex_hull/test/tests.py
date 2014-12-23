import subprocess
from subprocess import Popen, PIPE, STDOUT

convex_hull_app = "C:/projects/parallel/convex_hull/build/Release/convex_hull.exe"


def test ():
    rbox_process = Popen (['rbox', '10', 'D2'], stdout = PIPE)
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



if __name__ == "__main__":
    test ()
