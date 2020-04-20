from flask import Flask, render_template, jsonify
import gspn as pn

app = Flask(__name__)  # create an app instance


@app.route("/")
def home():
    return render_template("gspn_visualization_home.html", data=my_pn)


# background process happening without any refreshing
@app.route('/background_process_test')
def background_process_test():
    my_pn.simulate()
    print("Current Marking", my_pn.get_current_marking())
    return jsonify(my_pn.get_current_marking())


@app.route("/about")
def about():
    return render_template("gspn_visualization_about.html")


@app.route("/simulate_token_game")
def simulate_token_game():
    print("simulating!")
    my_pn.simulate(nsteps=10, simulate_wait=True)


if __name__ == "__main__":

    # Insert your GSPN inside this block
    my_pn = pn.GSPN()
    places = my_pn.add_places(['p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9', 'p10', 'p11', 'p12'],
                              [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1])
    trans = my_pn.add_transitions(['t1', 't2', 't3', 't4', 't5', 't6', 't7', 't8', 't9', 't10'],
                                  ['exp', 'exp', 'exp', 'exp', 'exp', 'imm', 'imm', 'exp', 'exp', 'exp'],
                                  [1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

    arc_in = {'p1': ['t1'], 'p2': ['t2'], 'p3': ['t3'], 'p5': ['t4'], 'p6': ['t4'], 'p7': ['t5'],
              'p8': ['t6', 't7'], 'p9': ['t8', 't9'], 'p10': ['t10'], 'p11': ['t4'], 'p12': ['t5']}

    arc_out = {'t1': ['p2'], 't2': ['p3'], 't3': ['p4', 'p5', 'p6'], 't4': ['p7'], 't5': ['p8', 'p9'], 't6': ['p1'],
               't7': ['p9'], 't8': ['p2'], 't9': ['p10'], 't10': ['p1']}

    a, b = my_pn.add_arcs(arc_in, arc_out)
    # End Insertion Block

    app.run(debug=True)
