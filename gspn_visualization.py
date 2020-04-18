from flask import Flask, render_template
import gspn as pn

app = Flask(__name__)  # create an app instance

@app.route("/")
def home():
    return render_template("gspn_visualization_home.html", data=my_pn)


@app.route("/about")
def about():
    return render_template("gspn_visualization_about.html")


if __name__ == "__main__":  # on running python app.py
    my_pn = pn.GSPN()
    places = my_pn.add_places(['p1', 'p2', 'p3', 'p4'], [3, 0, 0, 0])
    trans = my_pn.add_transitions(['t1'], ['exp'], [1])
    arc_in = {'p1': ['t1'], 'p2': ['t1']}
    arc_out = {'t1': ['p3', 'p4']}
    a, b = my_pn.add_arcs(arc_in, arc_out)

    app.run(debug=True)
