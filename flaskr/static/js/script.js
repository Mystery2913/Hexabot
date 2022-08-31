
function post(y) {

    fetch('/data', {
        headers : {
            'Content-Type' : 'application/json'
        },
        method : 'POST',
        body : JSON.stringify( {
            'y' : y
        })
    })
    .then(function (response){

        if(response.ok) {
            response.json()
            .then(function(response) {
                console.log(response);
            });
        }
        else {
            throw Error('Something went wrong');
        }
    })
    .catch(function(error) {
        console.log(error);
    });
};
