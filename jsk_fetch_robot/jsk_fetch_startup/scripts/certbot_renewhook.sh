#!/bin/sh

SITE=dialogflow.$(hostname).jsk.imi.i.u-tokyo.ac.jp

cd /etc/letsencrypt/live/$SITE
cat fullchain.pem privkey.pem > /etc/haproxy/certs/$SITE.pem
systemctl restart haproxy
