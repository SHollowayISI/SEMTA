import Tracking
from flask import Flask, request, flash
import os
import traceback

# Get local file path
dir_path = os.path.dirname(os.path.realpath(__file__))

# Setting variables
uploadFolder = dir_path + '\Input'

# Create directory if nonexistent
if not os.path.exists(uploadFolder):
    os.makedirs(uploadFolder)

# Set up flask server
app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = uploadFolder
app.config['SECRET_KEY'] = 'test'

# HTTP POST handler
@app.route('/', methods=['POST', 'GET'])
def uploadFile():
    
    # GET test response
    if request.method == 'GET':
        return 'Successfully pinged server.', 200

    # POST route to accept 
    elif request.method == 'POST':
        
        # Check if POST request has file attached
        if 'file' not in request.files:
            return 'ERROR: No file attached.', 400
        
        # Get file
        file = request.files['file']

        # Check for empty username
        if file.filename == '':
            return 'ERROR: No file attached.', 400

        # Check for file overwrite
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        if os.path.exists(filepath):
            return 'ERROR: File already exists with that name.', 403

        # Otherwise save to file
        file.save(filepath)

        try:
            Tracking.ProcessFile(filepath)
        except Exception as e:
            # Remove failed file
            os.remove(filepath)

            # Save error message
            errorMsg = traceback.format_exc()
            outputMsg =  (
            'ERROR: Upload accepted but could not process file.\n'
            'See Python error traceback below:\n\n')
            return outputMsg + errorMsg, 500

        return 'File successfully processed!', 201