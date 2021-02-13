# This is a sample Python script.
import collections

from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model


# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
# Create the linear solver with the GLOP backend.

def basic_linear(name):
    solver = pywraplp.Solver.CreateSolver('GLOP')

    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

    # Create the variables x and y.
    x = solver.NumVar(0, 1, 'x')
    y = solver.NumVar(0, 2, 'y')

    print('Number of variables =', solver.NumVariables())

    ct = solver.Constraint(0, 2, 'ct')
    ct.SetCoefficient(x, 1)
    ct.SetCoefficient(y, 1)

    print('Number of constraints =', solver.NumConstraints())

    objective = solver.Objective()
    objective.SetCoefficient(x, 3)
    objective.SetCoefficient(y, 1)
    objective.SetMaximization()

    solver.Solve()
    print('Solution:')
    print('Objective value =', objective.Value())
    print('x =', x.solution_value())
    print('y =', y.solution_value())


# https://developers.google.com/optimization/scheduling/job_shop
def flexible_job_shop_problem():
    """Minimal jobshop problem."""
    # Create the model.
    model = cp_model.CpModel()

    # Data
    jobs_data = [
        [3, 2, 2],
        [2, 1, 4],
        [4, 3]
    ]

    # Create 3 machines
    machines_count = 3
    all_machines = range(machines_count)

    # Variables

    # Computes horizon dynamically as the sum of all durations.
    horizon = sum(task for job in jobs_data for task in job)

    # Named tuple to store information about created variables.
    task_type = collections.namedtuple('task_type', 'start end interval')
    # Named tuple to manipulate solution information.
    assigned_task_type = collections.namedtuple('assigned_task_type',
                                                'start job index duration')

    # Creates job intervals and add to the corresponding machine lists.
    all_tasks = {}
    all_tasks_assigned = {}
    machine_to_intervals = collections.defaultdict(list)

    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            # add machine no overlap
            for machine_id, machine in enumerate(all_machines):
                duration = task
                suffix = '_%i_%i_%i' % (job_id, task_id, machine_id)
                start_var = model.NewIntVar(0, horizon, 'start' + suffix)
                end_var = model.NewIntVar(0, horizon, 'end' + suffix)
                assign = model.NewBoolVar('assigned' + suffix)
                interval_var = model.NewOptionalIntervalVar(start_var, duration, end_var, assign,
                                                            'interval' + suffix)
                machine_to_intervals[machine].append(interval_var)

                all_tasks[job_id, task_id, machine_id] = task_type(
                    start=start_var, end=end_var, interval=interval_var)
                all_tasks_assigned[job_id, task_id, machine_id] = assign

    for machine in enumerate(all_machines):
        model.AddNoOverlap(machine_to_intervals[machine])

    # Task only assigned to one machine
    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            model.Add(sum(all_tasks_assigned[job_id, task_id, machine_id] for machine_id in all_machines) == 1)

    # Precedences inside a job.
    for job_id, job in enumerate(jobs_data):
        for task_id in range(len(job) - 1):
            for machine_id in range(len(all_machines)):
                for machine_id1 in range(len(all_machines) - machine_id):
                    model.Add(all_tasks[job_id, task_id + 1, machine_id1].start
                              >= all_tasks[job_id, task_id, machine_id].end)

    # Makespan objective.
    obj_var = model.NewIntVar(0, horizon, 'makespan')
    for machine_id, machine in enumerate(all_machines):
        model.AddMaxEquality(obj_var, [
            all_tasks[job_id, len(job) - 1, machine_id].end
            for job_id, job in enumerate(jobs_data)
        ])
    model.Minimize(obj_var)

    # Solve model.
    solver = cp_model.CpSolver()
    solution_printer = SolutionPrinter()
    status = solver.SolveWithSolutionCallback(model, solution_printer)

    # Create one list of assigned tasks per machine.
    assigned_jobs = collections.defaultdict(list)
    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            for machine_id, machine in enumerate(all_machines):
                assigned_jobs[machine].append(
                    assigned_task_type(
                        start=solver.Value(all_tasks[job_id, task_id, machine_id].start),
                        job=job_id,
                        index=task_id,
                        duration=task))

    # Create per machine output lines.
    output = ''
    for machine in all_machines:
        # Sort by starting time.
        assigned_jobs[machine].sort()
        sol_line_tasks = 'Machine ' + str(machine) + ': '
        sol_line = '           '

        for assigned_task in assigned_jobs[machine]:
            name = 'job_%i_%i' % (assigned_task.job, assigned_task.index)
            # Add spaces to output to align columns.
            sol_line_tasks += '%-10s' % name

            start = assigned_task.start
            duration = assigned_task.duration
            sol_tmp = '[%i,%i]' % (start, start + duration)
            # Add spaces to output to align columns.
            sol_line += '%-10s' % sol_tmp

        sol_line += '\n'
        sol_line_tasks += '\n'
        output += sol_line_tasks
        output += sol_line

    # Finally print the solution found.
    print('Optimal Schedule Length: %i' % solver.ObjectiveValue())
    print(output)

    # Statistics.
    print('Statistics')
    print('  - conflicts       : %i' % solver.NumConflicts())
    print('  - branches        : %i' % solver.NumBranches())
    print('  - wall time       : %f s' % solver.WallTime())


class SolutionPrinter(cp_model.CpSolverSolutionCallback):
    """Print intermediate solutions."""

    def __init__(self):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.__solution_count = 0

    def on_solution_callback(self):
        """Called at each new solution."""
        print('Solution %i, time = %f s, objective = %i' %
              (self.__solution_count, self.WallTime(), self.ObjectiveValue()))
        self.__solution_count += 1


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # print_hi('PyCharm')
    flexible_job_shop_problem()
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
