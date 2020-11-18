<b><h1 align="center">Integração de comunicação TCP no esp8266 realizando a leitura dos sensores DHT11 e ultrassônico.</h1></b>
<ul><li>Materiais utilizados:</li></ul>
<b><span>DHT11:</span></b>
<img src="https://user-images.githubusercontent.com/25934169/99320149-c0f5f400-2849-11eb-94f5-eafc11ca55b6.png">
<b><span>Ultrassônico:</span></b>
<img src="https://user-images.githubusercontent.com/25934169/99320344-319d1080-284a-11eb-8ee2-4333e29d3e7a.png">
<b><span>ESP8266 (WeMos):</span></b>
<img src="https://user-images.githubusercontent.com/25934169/99320984-8db46480-284b-11eb-9b50-a004c0c9f477.png">
<ul><li>Montagem:</li></ul>
<table>
  <thead>
    <th>Pino Sensor</th>
	<th>Entrada Placa</th>
    <th>Tipo</th>
  </thead>
  <tbody>
    <tr>
	    <td>Comunicação</td>
	    <td>D5</td>
      <td>DHT11</td>
	 </tr>
    <tr>
	    <td>Echo</td>
	    <td>D2</td>
      <td>Ultrassônico</td>
	 </tr>
   <tr>
	    <td>Trigger</td>
	    <td>D3</td>
      <td>Ultrassônico</td>
	 </tr>
       <tr>
	    <td>Botão</td>
	    <td>D5</td>
      <td>Botão para reconectar wi-fi</td>
	 </tr>
  </tbody>
</table>
<ul><li>Configuração:</li></ul>

<table>
  <thead>
    <th>Paramêtro</th>
   	<th>Dado</th>
  </thead>
  <tbody>
    <tr>
	    <td>WiFi SSID</td>
	    <td>Nome da rede wi-fi</td>
	 </tr>
    <tr>
	    <td>WiFi Password</td>
	    <td>Senha da rede wi-fi</td>
	 </tr>
   <tr>
	    <td>Maximum Retry</td>
	    <td>Quantidade de tentativas de conexão</td>
	 </tr>
   <tr>
	    <td>Port</td>
	    <td>Porta do socket</td>
	 </tr>
  </tbody>
</table>

<ul><li>Uso:</li></ul>
<p>Para a utilização deve-se conectar um cliente no endereço fornecido pela rede ao WeMos utilizando a porta definida.
<p>Os comandos que podem ser utilizados são:</p>

<table>
  <thead>
    <th>Comando</th>
   	<th>Resposta</th>
  </thead>
  <tbody>
    <tr>
	    <td>TEMP</td>
	    <td>Retorna a temperatura lida pelo sensor de temperatura</td>
	 </tr>
   <tr>
	    <td>DIST</td>
	    <td>Retorna a distancia lida pelo sensor ultrassônico</td>
	 </tr>
  </tbody>
</table>
