import os
import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader, random_split
from sklearn.metrics import confusion_matrix
import seaborn as sns
import matplotlib.pyplot as plt

"""
Non-OOP code that generates a model to use for object detection
"""

# change images to add diversity
# this should help to avoid overfitting
transform = transforms.Compose([
    transforms.RandomResizedCrop(224),
    transforms.RandomHorizontalFlip(),
    transforms.RandomRotation(degrees=15),
    transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])


data_dir = '/home/vlois/catkin_ws/src/intro_robo/robotics-final-project/scripts/turtlebot_images'
dataset = datasets.ImageFolder(data_dir, transform=transform)

# set training dataset size 
train_size = int(0.7 * len(dataset))
val_size = len(dataset) - train_size
# randomly set dataset
train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

# load datat from folders
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)

model = models.resnet18(pretrained=True)
num_ftrs = model.fc.in_features
model.fc = nn.Linear(num_ftrs, 3)  

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model = model.to(device)

criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(model.parameters(), lr=0.6, momentum=0.7, weight_decay=.001)
# run 5 iteratiosn
num_epochs = 5

for epoch in range(num_epochs):
    model.train()
    running_loss = 0.0

    for inputs, labels in train_loader:
        inputs, labels = inputs.to(device), labels.to(device)

        optimizer.zero_grad()

        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        running_loss += loss.item() * inputs.size(0)
# print stats
    epoch_loss = running_loss / train_size
    print(f'Epoch {epoch}/{num_epochs - 1}, Loss: {epoch_loss:.4f}')

# validation
    model.eval()
    val_loss = 0.0
    corrects = 0
    # validate model
    with torch.no_grad():
        for inputs, labels in val_loader:
            inputs, labels = inputs.to(device), labels.to(device)

            outputs = model(inputs)
            loss = criterion(outputs, labels)
            val_loss += loss.item() * inputs.size(0)

            _, preds = torch.max(outputs, 1)
            corrects += torch.sum(preds == labels.data)
# calculate loss and accuracy
    val_loss = val_loss / val_size
    val_acc = corrects.double() / val_size
    print(f'Validation Loss: {val_loss:.4f}, Accuracy: {val_acc:.4f}')

torch.save(model.state_dict(), 'soda_classifier.pt')


y_true = []
y_pred = []
# generate confusion matrix to see TPs
with torch.no_grad():
    for inputs, labels in val_loader:
        inputs, labels = inputs.to(device), labels.to(device)
        outputs = model(inputs)
        _, preds = torch.max(outputs, 1)
        y_true.extend(labels.cpu().numpy())
        y_pred.extend(preds.cpu().numpy())

cm = confusion_matrix(y_true, y_pred)
plt.figure(figsize=(10, 8))
sns.heatmap(cm, annot=True, fmt='d', cmap='Blues')
plt.xlabel('Predicted')
plt.ylabel('True')
plt.show()
