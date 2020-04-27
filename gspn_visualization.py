from flask import Flask, render_template, jsonify, request
import gspn as pn

app = Flask(__name__)  # create an app instance


@app.route("/")
def home():
    return render_template("gspn_visualization_home.html", data=my_pn)


@app.route('/background_process_test')
def background_process_test():
    my_pn.simulate()
    print("Current Marking", my_pn.get_current_marking())
    return jsonify(my_pn.get_current_marking())


@app.route('/background_reset_simulation')
def background_reset_simulation():
    my_pn.reset_simulation()
    return jsonify(my_pn.get_current_marking())


@app.route('/background_check_liveness')
def background_check_liveness():
    my_pn.init_analysis()
    liveness_check = my_pn.liveness()
    return jsonify(liveness_check)


@app.route('/background_check_throughputrate', methods=['GET', 'POST'])
def background_check_throughputrate():
    my_pn.init_analysis()
    text = request.form.get('throughput_rate_text', None)
    processed_text = str(text)
    throughput = my_pn.transition_throughput_rate(processed_text)
    return jsonify("to-do")


@app.route('/background_check_probntokens', methods=['GET', 'POST'])
def background_check_probntokens():
    my_pn.init_analysis()
    text = request.form.get('prob_n_tokens_text', None)
    processed_text = str(text)
    return jsonify("to-do")


@app.route('/background_check_exepectedntokens', methods=['GET', 'POST'])
def background_check_expectedntokens():
    my_pn.init_analysis()
    text = request.form.get('expected_n_tokens_text', None)
    processed_text = str(text)
    return jsonify("to-do")


@app.route('/background_check_transitionprobevo', methods=['GET', 'POST'])
def background_check_transitionprobevo():
    my_pn.init_analysis()
    text = request.form.get('transition_prob_evo_text', None)
    processed_text = str(text)
    return jsonify("to-do")


@app.route('/background_check_meanwaittime', methods=['GET', 'POST'])
def background_check_meanwaittime():
    my_pn.init_analysis()
    text = request.form.get('mean_wait_time_text', None)
    processed_text = str(text)
    return jsonify("to-do")


@app.route("/about")
def about():
    return render_template("gspn_visualization_about.html")


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
