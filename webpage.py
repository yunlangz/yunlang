from flask import Flask, render_template_string


class WepPage:
    app = Flask(__name__)

    html = """<!DOCTYPE html>
    <html lang="en">
     
    <head>
      <title>Map</title>
    </head>
     
    <body>
      Map: <BR>
     <img src="{{url_for('static', filename='images/map.png')}}">
    </body>
     
    </html>"""

    @staticmethod
    @app.route("/")
    def home():
        return render_template_string(WepPage.html)

    @staticmethod
    def run():
        WepPage.app.run(debug=True)

if __name__ == "__main__":
    WepPage.app.run(debug=True)
