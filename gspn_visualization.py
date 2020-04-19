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


if __name__ == "__main__":  # on running python app.py
    my_pn = pn.GSPN()
    places = my_pn.add_places(['p1', 'p2', 'p3', 'p4'], [1, 1, 0, 1])
    trans = my_pn.add_transitions(['t1'], ['exp'], [1])
    arc_in = {'p1': ['t1'], 'p2': ['t1']}
    arc_out = {'t1': ['p3', 'p4']}
    a, b = my_pn.add_arcs(arc_in, arc_out)

    app.run(debug=True)
