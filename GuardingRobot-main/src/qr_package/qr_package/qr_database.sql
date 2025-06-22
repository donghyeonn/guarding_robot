DROP DATABASE IF EXISTS qr_database;
CREATE DATABASE qr_database;
USE qr_database;

CREATE TABLE IF NOT EXISTS marker_info (
    id INT PRIMARY KEY,
    name VARCHAR(50),
    description TEXT
);

INSERT INTO marker_info (id, name, description) VALUES
(1, 'a', 'This artwork is The last supper.'),
(2, 'b', 'This artwork is sunflower.'),
(3, 'c', 'This artwork is mona lisa and it  is a famous portrait painted by Leonardo da Vinci in the early 16th century. It is renowned for the subject’s mysterious smile and the painting’s detailed background. .'),
(4, 'd', 'This artwork is The creation of Adam.'),
(5, 'e', 'This artwork is statue of liberty.'),
(6, 'f', 'This artwork is The starry night.'),	
(7, 'g', 'This artwork is A Sunday on La Grande Jatte.');