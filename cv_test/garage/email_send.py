#!/usr/bin/env python

import sys, os, json
# Import smtplib for the actual sending function
import smtplib
# Import the email modules we'll need
from email.mime.base import MIMEBase
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
from email.mime.application import MIMEApplication
from email import encoders
from email.parser import HeaderParser
from email.utils import formatdate

from email_credential import *

def send_email(addr_to, subj, body, imagedata = []):

	msg = MIMEMultipart()
	msg['Subject'] = subj
	msg['From'] = addr_from
	msg['To'] = addr_to
	msg.attach(MIMEText(body, 'plain'))
	for idata in imagedata:
		msg.attach(MIMEImage(idata))
	s = smtplib.SMTP(smtp_host, int(smtp_port))
#	s.set_debuglevel(True)
	s.starttls()
	s.login(username, userpass)
	failures = s.sendmail(addr_from, addr_to, msg.as_string())
	s.quit()
	print failures


if __name__ == '__main__':

	send_email('n800sau@gmail.com', 'Test', 'hello', [])
