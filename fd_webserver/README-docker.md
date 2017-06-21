# Run
`docker run --restart always -p 5000:5000 --name "fast-downward" -it  strands/fd_server`

point browser to http://localhost:5000/fast-downward/

# Build
`docker build --network host -t strands/fd_server .`
