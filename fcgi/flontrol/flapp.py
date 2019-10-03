from flask import Flask, render_template, request, url_for, Blueprint, abort
from jinja2 import TemplateNotFound

app = Flask(__name__)
app.debug = True

simple_page = Blueprint('simple_page', __name__, template_folder='templates')
app.register_blueprint(simple_page)
# Blueprint can be registered many times
app.register_blueprint(simple_page, url_prefix='/robo')

#@app.route('/robo')
def home():
	return render_template('home.html')

#@app.route('/robo/histogram', methods=['POST', 'GET'])
def histogram():
	return render_template('histogram.html')

#@app.route('/robo/login', methods=['POST', 'GET'])
def login():
    error = None
    if request.method == 'POST':
        if valid_login(request.form['username'],
                       request.form['password']):
            return log_the_user_in(request.form['username'])
        else:
            error = 'Invalid username/password'
    # the code below this is executed if the request method
    # was GET or the credentials were invalid
    return render_template('login.html', error=error)

@app.errorhandler(404)
def not_found(error):
	print '##########', request
	return render_template('error.html', request=request), 404



@simple_page.route('/', defaults={'page': 'index'})
@simple_page.route('/<page>/<page1>')
@simple_page.route('/<page>')
def show(page, page1=None):
#    try:
        return render_template('home.html', page=page)
#    except TemplateNotFound:
#        abort(404)



if __name__ == '__main__':
	app.run()

