import subprocess
from subprocess import Popen, PIPE, STDOUT

def test ():
    rbox_process = Popen (['rbox', '100', 'D2'], stdout = PIPE)
    problem = rbox_process.communicate ()
    print (problem[0])
    
    qh = subprocess.Popen(['qhull', 'p'], stdout=PIPE, stdin=PIPE, stderr=STDOUT)

    qhull_output = qh.communicate (input=problem[0])[0]
    print ("Qhull Result:\n", qhull_output)

if __name__ == "__main__":
    test ()
