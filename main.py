import os, time, threading
from flask import Flask, request, render_template, redirect, url_for
from werkzeug.utils import secure_filename

from tracing.tracing import tracing
from control.platform import Platform


print("connecting to the robot...")
p = Platform('192.168.12.20')
platform_action = 0  # 0-stop/1-forward/2-backward/3-left/4-right

UPLOAD_FOLDER = 'C:/images'
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg', 'gif'}

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

image_loaded = False  # True - image loaded
tracing_method = True  # True - algorithm / False - neuronet
shading = False  # True - enabled / False - disabled


def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


@app.route('/', methods=['GET', 'POST'])
def menu():
    if request.method == 'POST':
        action = request.form['actions']
        if action == 'drive robot':
            return redirect(url_for('control'))
        if action == 'choose tracing method':
            return redirect(url_for('select_tracing'))
        if action == 'upload image':
            return redirect(url_for('upload'))
        if action == 'show preview':
            return redirect(url_for('preview'))
        if action == 'start drawing':
            return redirect(url_for('select_size'))

    if not image_loaded:
        return render_template('menu.html', enabled='disabled')
    return render_template('menu.html', enabled=' ')


@app.route('/upload', methods=['GET', 'POST'])
def upload():
    if request.method == 'POST':
        # check if the post request has the file part
        # if 'file' not in request.files:
        #     flash('No file part')
        #     return 'not ok'

        if request.form['buttons'] == '<<back':
            return redirect(url_for('menu'))
        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if file.filename == '':
            # flash('No selected file')
            return render_template('upload_page.html')
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
            global image_loaded
            if tracing_method:
                image_loaded = True
                contours = tracing(filename, shading=shading)
            else:
                return 'Neuronet is not available. Choose another tracing method please.'
            return redirect(url_for('menu'))
    return render_template('upload_page.html')


@app.route('/preview', methods=['GET', 'POST'])
def preview():
    if request.method == 'POST':
        return redirect(url_for('menu'))

    filename = os.path.join('static', "preview.jpg")
    return render_template('preview.html', preview_image=filename)


@app.route('/control', methods=['GET', 'POST'])
def control():
    global platform_action
    if request.method == 'POST':
        button = request.form['drive']
        print(button)
        if button == 'forward':
            p.drive(0.1, 0.0)
            platform_action = 1
        if button == 'backward':
            p.drive(-0.1, 0.0)
            platform_action = 2
        if button == 'left':
            p.drive(0.0, 0.1)
            platform_action = 3
        if button == 'right':
            p.drive(0.0, -0.1)
            platform_action = 4
        if button == 'stop':
            p.drive(0.0, 0.0)
            platform_action = 0

        if button == '<<back':
            p.drive(0.0, 0.0)
            platform_action = 0
            return redirect(url_for('menu'))

    return render_template('control.html')


@app.route('/select_size', methods=['GET', 'POST'])
def select_size():
    if request.method == 'POST':
        if request.form['confirm'] == 'confirm':
            width = request.form['width']
            height = request.form['height']
            print('width ' + width)
            print('height ' + height)
        else:
            return redirect(url_for('menu'))
    return render_template('select_size.html')


@app.route('/select_tracing', methods=['GET', 'POST'])
def select_tracing():
    if request.method == 'POST':
        global tracing_method
        global shading
        if request.form['select'] == 'neuronet':
            # print('neuronet tracing')
            tracing_method = False
        else:
            # print('algorithm tracing')
            tracing_method = True
        if 'shading' in request.form:
            # print('shading enabled')
            shading = True
        else:
            shading = False
        return redirect(url_for('menu'))
    return render_template('tracing_method.html')


# def control():
#     global platform_action
#     while True:
#         if platform_action != 0:
#             print(platform_action)
#
#         if platform_action == 0:
#             p.drive(0.0, 0.0)
#         elif platform_action == 1:
#             p.drive(0.1, 0.0)
#         elif platform_action == 2:
#             p.drive(-0.1, 0.0)
#         elif platform_action == 3:
#             p.drive(0.0, 0.1)
#         elif platform_action == 4:
#             p.drive(0.0, -0.1)


def flask_start():
    app.run(host="0.0.0.0", port=5000)


if __name__ == '__main__':
    # t1 = threading.Thread(target=control)
    # t1.start()
    # t2 = threading.Thread(target=flask_start())
    # t2.start()
    app.run(host="0.0.0.0", port=5000)
    while True:
        pass
    # app.run(host="0.0.0.0", port=5000)
