const chat = document.getElementById("chat");
const usernameInput = document.getElementById("username");
const messageInput = document.getElementById("message");
const sendButton = document.getElementById("send");

sendButton.addEventListener("click", () => {
    const username = usernameInput.value.trim();
    const messageText = messageInput.value.trim();
    
    if (username === "" || messageText === "") {
        alert("Пожалуйста, введите имя и сообщение.");
    } else {
        appendMessage(username, messageText);
        messageInput.value = "";
    }
});

function appendMessage(sender, text) {
    const messageDiv = document.createElement("div");
    messageDiv.classList.add("message");
    messageDiv.textContent = `${sender}: ${text}`;
    chat.appendChild(messageDiv);
}
