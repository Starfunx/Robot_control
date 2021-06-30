from task_manager import Task
import time

#1. un objet TASK ne peux reasliser sa tâche qu'une seule fois !!!!
#2. il prend en param de contrustucteur la fonction qui réalise la tache
#3. La tache doit etre démarée avec la methode start()
#3. La tache peut être interompu définitivement avec la methode stop()
#4. pour verifier une si une tache est terminée lire la valeur de l'arguement done

Tmax = 13

A = 0
############################
# Tasks definition

# compter jusqu'à 10
def compte():
    global A
    while A < 10:
        A = A + 1
        print(A)
        time.sleep(1)


task1 = Task(compte)



############################
# wait for start

############################
# start
t0 = time.time()
t = t0
# Main loop
while t < t0+Tmax:
    
    task1.start()

    if task1.done:
        pass

    t = time.time()

task1.stop()


time.sleep(2)
print("fin")