## SSL Certificates

This folder contains some SSL certificates and keys that are used **ONLY FOR TESTING PURPOSES**.

These keys and certificates were generated using `openssl`. The file `test_authority.ca.crt` is meant to be used as a certificate authority for test clients that want to talk to a test server. To create your own certificate that is signed by the test authority, run the following commands from this directory:

```
$ openssl genrsa -out my_test.key 4096
$ openssl req -new -config openssl.conf -key my_test.key -out my_test.csr
$ openssl x509 -sha256 -req -in my_test.csr -CA test_authority.ca.crt -CAkey test_authority.ca.key -CAcreateserial -out my_test.crt -days 3650 -extensions v3_req -extfile openssl.conf
```