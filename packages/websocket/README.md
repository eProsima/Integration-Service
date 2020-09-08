## Configuration

The following fields are available:

*type*: REQUIRED. MUST be `websocket_server` to use this plugin.

*port*: REQUIRED. The port to listen on.

*cert*: REQUIRED. x509 cert the server should use. Currently the server only supports wss protocol so this field is required.

*key*: REQUIRED. private key corresponding to the cert.

*authentication*: see [Authentication Options](#Authentication-Options)

Example:
```yaml
systems:
  client:
    type: websocket_server
    port: 50001
    cert: path/to/soss.crt
    key: path/to/soss.key
    authentication: { secret: my_secret }

...other soss options
```

Paths for cert and keys can be either absolute, relative to the config file or relative to your home directory.

### Authentication Options

The authentication option is a map with the following keys:

*secret*: The secret to use for MAC based verification.

*pubkey*: Path to a file containing a PEM encoded public key.

Either a *secret* or *pubkey* must be present.

*rules*: List of additional claims that should be checked. It should contain a map with keys corresponding to the claim identifier and value a glob pattern.

For example, this rule will check if the payload contains a `myClaim` claim and that it's value is exactly equal to `expectedValue`.
```yaml
rules: { myClaim: expectedValue }
```
And this rule will check that `myClaim` contains any value that starts will `foo`
```yaml
rules: { myClaim: foo* }
```

*policies*: An array of authentication options to check, a token is allowed if it passes any of the policies. Each policy is an authentication option, that is, it should have the same fields as described above (*secret*, *pubkey*, *rules* etc). If this field is present, the default policy will not be used.

For example, this will allow a token that is signed with `my_secret` OR a token that is signed with `my_secret_2` and has a `myClaim` claim with the value `myValue`.
```yaml
policies: [
  {
    secret: my_secret,  
  },
  {
    secret: my_secret_2,
    rules: { myClaim: myValue },  
  },
]
```
In this example, the default policy here will NOT be used, so this will only accept tokens signed with `my_secret_2`.
```yaml
secret: my_secret # not used!
policies: [ { secret: my_secret_2 }]
```
