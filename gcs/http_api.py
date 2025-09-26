from flask import Flask, request, jsonify
app = Flask(__name__)


@app.route('/health')
def health():
    return jsonify(ok=True)
