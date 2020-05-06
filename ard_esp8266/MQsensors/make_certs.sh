sudo mkdir /etc/nginx/ssl
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 9999 && \
openssl rsa -in key.pem -out key.pem && \
sudo cp key.pem cert.pem /etc/nginx/ssl && \
sudo chmod 600 /etc/nginx/ssl/key.pem /etc/nginx/ssl/cert.pem && \
sudo chown root:root /etc/nginx/ssl/key.pem /etc/nginx/ssl/cert.pem && \
cd /etc/nginx/ssl && \
sudo openssl dhparam -out dhparams.pem 2048
echo $?

