from flask import Flask, render_template, request, jsonify, Blueprint
from subprocess import CalledProcessError

app = Flask(__name__)

bp = Blueprint('fd_webserver', __name__, static_folder='static')

def call_planner(domain, problem):

    from driver.main import main
    from tempfile import mkdtemp
    from shutil import rmtree
    from os import path

    tmpdir = mkdtemp()

    domain_file = path.join(tmpdir, 'domain.pddl')
    problem_file = path.join(tmpdir, 'problem.pddl')
    plan_file = path.join(tmpdir, 'plan.out')

    print "operate in %s" % tmpdir

    with open(domain_file, "w") as text_file:
        text_file.write(domain)
    with open(problem_file, "w") as text_file:
        text_file.write(problem)

    try:
        log = main(["--plan-file", plan_file, "--cwd", tmpdir, problem_file, "--search", "astar(ff)"])
        with open(plan_file, "r") as text_file:
            p = text_file.read()
        return log, p

    except (CalledProcessError) as e:
        return e.output, "no plan due to error. check logs"

    except (RuntimeError, OSError) as e:
        print e
        return str(e), "no plan due to error. check logs"

    rmtree(tmpdir, ignore_errors=True)
    return "This contains the logs", "This shall be the plan"


@bp.route('/', methods=['POST', 'GET'])
def index(name=None):
    if request.method == 'POST':
        print "post"
        domain = request.form['domain']
        problem = request.form['problem']
        sout, plan = call_planner(domain, problem)
        return jsonify(sout=sout, plan=plan)
    else:
        domain = request.args.get('domain', '')
        problem = request.args.get('problem', '')
        return render_template('index.html', domain=domain, problem=problem)


if __name__ == '__main__':
    app.register_blueprint(bp, url_prefix='/fast-downward')
    app.run(debug=True, threaded=True, host='0.0.0.0')
