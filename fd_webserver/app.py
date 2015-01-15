from flask import Flask, render_template, request, jsonify


app = Flask(__name__)


def call_planner(domain, problem):
    print "call the planner here: "
    print "Domain: %s" % domain
    print "Problem: %s" % problem
    return "This contains the logs", "This shall be the plan"


@app.route('/', methods=['POST', 'GET'])
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
    app.run(debug=True)
