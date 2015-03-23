Pinocchio Website
===================

Forked from roboptim.net

## Testing locally

In order to test your modifications locally before pushing to GitHub, install
`jekyll`, and run the following command at the root of the project folder:

```sh
$ jekyll server --safe --trace
```

You will get something like:

```
Configuration file: /path/to/pinocchio/website/_config.yml
           Source:  /path/to/pinocchio/website
       Destination: /path/to/pinocchio/website_site
      Generating... 
                    done.
Configuration file: /path/to/pinocchio/website/_config.yml
    Server address: http://0.0.0.0:4000/
  Server running... press ctrl-c to stop
```

Then simply type the server address in your browser (`http://0.0.0.0:4000/`
here).

Note: Check the base url in _config.yml

