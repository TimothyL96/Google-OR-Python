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

    print("Horizon:", horizon)
    #
    #
    #
    #
    #
    #
    #
    #
    #
    # Named tuple to store information about created variables.
    task_type = collections.namedtuple('task_type', 'start end interval assign')
    # Named tuple to manipulate solution information.
    assigned_task_type = collections.namedtuple('assigned_task_type',
                                                'start job index duration')

    # Creates job intervals and add to the corresponding machine lists.
    all_tasks = {}
    starts = {}
    machine_to_intervals = collections.defaultdict(list)
    job_ends = []

    for job_id, job in enumerate(jobs_data):
        previous_end = None
        for task_id, task in enumerate(job):
            l_assign = []

            min_duration = min(job)
            max_duration = max(job)

            # Create main interval for the task.
            suffix_name = '_j%i_t%i' % (job_id, task_id)
            start1 = model.NewIntVar(0, horizon, 'start1' + suffix_name)
            duration = model.NewIntVar(min_duration, max_duration,
                                       'duration' + suffix_name)
            end1 = model.NewIntVar(0, horizon, 'end1' + suffix_name)
            interval1 = model.NewIntervalVar(start1, duration, end1,
                                            'interval1' + suffix_name)

            starts[(job_id, task_id)] = start1

            if previous_end:
                model.Add(start1 >= previous_end)
            previous_end = end1

            for machine_id, machine in enumerate(all_machines):
                # add machine no overlap
                suffix = '_%i_%i' % (job_id, task_id)
                start_var = model.NewIntVar(0, horizon, 'start' + suffix)
                end_var = model.NewIntVar(0, horizon, 'end' + suffix)
                assign = model.NewBoolVar('assigned' + suffix)
                interval_var = model.NewOptionalIntervalVar(start_var, duration, end_var, assign,
                                                            'interval' + suffix)
                machine_to_intervals[machine].append(interval_var)

                model.Add(start1 == start_var).OnlyEnforceIf(l_assign)
                model.Add(end1 == end_var).OnlyEnforceIf(l_assign)

                l_assign.append(assign)
                all_tasks[job_id, task_id, machine_id] = assign

            model.Add(sum(l_assign) == 1)
        job_ends.append(previous_end)

    for machine in all_machines:
        model.AddNoOverlap(machine_to_intervals[machine])

    # Precedences inside a job.
    # for job_id, job in enumerate(jobs_data):
    #     for task_id in range(len(job) - 1):
    #         for machine_id in range(len(all_machines)):
    #             for machine_id1 in range(len(all_machines) - machine_id):
    #                 model.Add(all_tasks[job_id, task_id + 1, machine_id1].start
    #                           >= all_tasks[job_id, task_id, machine_id].end)

    #
    #
    #
    #
    #
    #
    #
    #
    #
    #
    #
    #
    # Makespan objective.
    obj_var = model.NewIntVar(0, horizon, 'makespan')
    model.AddMaxEquality(obj_var, job_ends)
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
                        start=solver.Value(starts[(job_id, task_id)]),
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


def flexible_jobshop():
    """Solve a small flexible jobshop problem."""
    # Data part.
    jobs1 = [ # task = (processing_time, machine_id)
        [ # Job 0
            [(3, 0), (1, 1), (5, 2)], # task 0 with 3 alternatives
            [(2, 0), (4, 1), (6, 2)], # task 1 with 3 alternatives
            [(2, 0), (3, 1), (1, 2)], # task 2 with 3 alternatives
        ],
        [ # Job 1
            [(2, 0), (3, 1), (4, 2)],
            [(1, 0), (5, 1), (4, 2)],
            [(2, 0), (1, 1), (4, 2)],
        ],
        [ # Job 2
            [(2, 0), (1, 1), (4, 2)],
            [(2, 0), (3, 1), (4, 2)],
            [(3, 0), (1, 1), (5, 2)],
        ],
    ]

    jobs = [
        [3, 2, 2],
        [2, 1, 4],
        [4, 3]
    ]

    num_jobs = len(jobs)
    all_jobs = range(num_jobs)

    num_machines = 3
    all_machines = range(num_machines)

    # Model the flexible jobshop problem.
    model = cp_model.CpModel()

    horizon = 0
    for job in jobs:
        max_task_duration = sum(job)
        horizon += max_task_duration

    print('Horizon = %i' % horizon)

    # Global storage of variables.
    intervals_per_resources = collections.defaultdict(list)
    starts = {}  # indexed by (job_id, task_id).
    presences = {}  # indexed by (job_id, task_id, alt_id).
    job_ends = []

    # Scan the jobs and create the relevant variables and intervals.
    for job_id in all_jobs:
        job = jobs[job_id]
        num_tasks = len(job)
        previous_end = None
        for task_id in range(num_tasks):
            task = job[task_id]

            min_duration = task
            max_duration = task

            # Create main interval for the task.
            suffix_name = '_j%i_t%i' % (job_id, task_id)
            start = model.NewIntVar(0, horizon, 'start' + suffix_name)
            duration = model.NewIntVar(min_duration, max_duration,
                                       'duration' + suffix_name)
            end = model.NewIntVar(0, horizon, 'end' + suffix_name)

            # Store the start for the solution.
            starts[(job_id, task_id)] = start

            # Add precedence with previous task in the same job.
            if previous_end:
                model.Add(start >= previous_end)
            previous_end = end

            # Create alternative intervals.
            l_presences = []
            for machine_id, machine_id in enumerate(all_machines):
                alt_suffix = '_j%i_t%i_a%i' % (job_id, task_id, machine_id)
                l_presence = model.NewBoolVar('presence' + alt_suffix)
                l_start = model.NewIntVar(0, horizon, 'start' + alt_suffix)
                l_duration = task
                l_end = model.NewIntVar(0, horizon, 'end' + alt_suffix)
                l_interval = model.NewOptionalIntervalVar(
                    l_start, l_duration, l_end, l_presence,
                    'interval' + alt_suffix)
                l_presences.append(l_presence)

                # Link the master variables with the local ones.
                model.Add(start == l_start).OnlyEnforceIf(l_presence)
                model.Add(duration == l_duration).OnlyEnforceIf(l_presence)
                model.Add(end == l_end).OnlyEnforceIf(l_presence)

                # Add the local interval to the right machine.
                intervals_per_resources[machine_id].append(l_interval)

                # Store the presences for the solution.
                presences[(job_id, task_id, machine_id)] = l_presence

            # Select exactly one presence variable.
            model.Add(sum(l_presences) == 1)

        job_ends.append(previous_end)

    # Create machines constraints.
    for machine_id in all_machines:
        intervals = intervals_per_resources[machine_id]
        if len(intervals) > 1:
            model.AddNoOverlap(intervals)

    # Makespan objective
    makespan = model.NewIntVar(0, horizon, 'makespan')
    model.AddMaxEquality(makespan, job_ends)
    model.Minimize(makespan)

    # Solve model.
    solver = cp_model.CpSolver()
    solution_printer = SolutionPrinter()
    status = solver.SolveWithSolutionCallback(model, solution_printer)

    # Print final solution.
    for job_id in all_jobs:
        print('Job %i:' % job_id)
        for task_id in range(len(jobs[job_id])):
            start_value = solver.Value(starts[(job_id, task_id)])
            duration = -1
            selected = -1
            for machine_id, machine in enumerate(all_machines):
                if solver.Value(presences[(job_id, task_id, machine_id)]):
                    duration = jobs[job_id][task_id]
                    selected = machine_id
            print(
                '  task_%i_%i starts at %i (machine %i, duration %i, end %i)' %
                (job_id, task_id, start_value, selected, duration, start_value+duration))

    print('Solve status: %s' % solver.StatusName(status))
    print('Optimal objective value: %i' % solver.ObjectiveValue())
    print('Statistics')
    print('  - conflicts : %i' % solver.NumConflicts())
    print('  - branches  : %i' % solver.NumBranches())
    print('  - wall time : %f s' % solver.WallTime())


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # print_hi('PyCharm')
    flexible_jobshop()
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
