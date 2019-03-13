var modal = document.getElementById('myModal');
var btn = document.getElementById("myBtn");
var order = document.getElementById("sendOrder");
// Get the <span> element that closes the modal
var span = document.getElementsByClassName("close")[0];
var modal1 = document.getElementById('myModal1');

// When the user clicks the button, open the modal 
btn.onclick = function() {
  modal.style.display = "block";
}
// When the user clicks on <span> (x), close the modal
span.onclick = function() {
  modal.style.display = "none";
}
// When the user clicks anywhere outside of the modal, close it
window.onclick = function(event) {
  if (event.target == modal) {
    modal.style.display = "none";
  }
}
order.onclick = function() {
  SendOrder();
  modal.style.display = "none";
}

function PlasticPay(pic, name, product, price) {
  document.getElementById('customPic').src      = pic;
  document.getElementById('customName').value   = name;
  document.getElementById('productName').value  = product;
  document.getElementById('productPrice').value = price;
  modal1.style.display = "block";
  setTimeout(function () { modal1.style.display = "none"; }, 2000);
}