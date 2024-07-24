import time
from datetime import datetime
from multiprocessing import Process

import usb.core
import usb.util

import certifi
import smtplib, email

from pymongo import MongoClient
from pymongo.errors import PyMongoError

from io_handling import log_error, get_email_creds

__all__ = ['wait_for_usb_connect', 'usb_conn_lost_wrapper',
           'login_to_database', 'try_database_upload', 'EmailServerLink']


### USB CONNECTION HANDLING ###

def wait_for_usb_connect(initial_notice: bool = False):
    if initial_notice:
        print('Waiting for USB connection to wearout test controller...')
    dev = None
    while dev is None:
        time.sleep(2)
        dev = usb.core.find(idVendor=0x7076, idProduct=0x0005)
        if dev is not None:
            try:
                print('Device found, configuring...')
                dev.set_configuration()
                time.sleep(2)
            except Exception as e:
                dev = None
                print(e)
                if 'Resource busy' in str(e):
                    print('USB channel busy')
                    continue
                print('Configuration failed, awaiting connection...')
    print('Connection to controller established!')
    time.sleep(1)
    return dev


def usb_conn_lost_wrapper(error, em_srv):
    log_error(error)
    mail_msg = f"Connection to controller lost at {datetime.utcnow()}."
    Process(target=em_srv.build_and_send_email, args=(mail_msg,)).start()
    dev = wait_for_usb_connect(initial_notice=True)
    mail_msg = f"Connection to controller re-established at {datetime.utcnow()}."
    Process(target=em_srv.build_and_send_email, args=(mail_msg,)).start()
    return dev


### EMAIL SEND HANDLING ###

TARGET_EMAIL = 'mateorendonarias777@gmail.com'
DEVICE_NAME = 'Weartest Controller #1'

class EmailServerLink:
    def __init__(self):
        self.address, self.password = get_email_creds()
        print('Connecting to email server...')
        try:
            with smtplib.SMTP_SSL('smtp.gmail.com', 465, timeout=2) as server:
                server.login(self.address, self.password)
                server.quit()
            print('Email server connection established...')
        except Exception as e:
            print('Email server is not correctly configured, exiting...')
            print(e)
            exit()

    def build_and_send_email(self, msg):
        mail = email.message.EmailMessage()
        mail['From'] = DEVICE_NAME
        mail['To'] = TARGET_EMAIL
        mail['Subject'] = 'Test Status Update Email'
        mail.set_content(msg)
        try:
            with smtplib.SMTP_SSL('smtp.gmail.com', 465, timeout=2) as server:
                server.login(self.address, self.password)
                # TEMP TODO: Uncomment after testing
                #server.sendmail(source_email, TARGET_EMAIL, mail.as_string())
                print('Email would have sent successfully')
                server.quit()
        except Exception as e:
            print(f"Email send failed: {self.address}, {self.password}")
            log_error(e)


### DATABASE CONNECTION HANDLING ###

DB_DATASET = 'IDFBCAMQ-temp-test'

def login_to_database():
    tls_ca = certifi.where()
    uri = "mongodb+srv://arbutus.6v6mkhr.mongodb.net/?authSource=%24external&authMechanism=MONGODB-X509&retryWrites=true&w=majority"
    mongo_client = MongoClient(uri, tls=True, tlsCertificatekeyFile='mongo_cert.pem', tlsCAFile=tls_ca)
    db = mongo_client['weartest']
    dataset = db[DB_DATASET]
    try:
        mongo_client.admin.command('ping')
    except PyMongoError as e:
        print(e)
        print("\nCould not connect to database successfully, exiting...")
        exit()
    return dataset


def try_database_upload(dataset, formatted_data, report_error=True, em_srv=None):
    try:
        dataset.insert_one(formatted_data)
        return None
    except PyMongoError as e:
        if report_error:
            log_error(e)
            mail_msg = f"Encountered error trying to upload data to database at {datetime.utcnow()}"
            Process(target=em_srv.build_and_send_email, args=(mail_msg,)).start()
        return formatted_data

